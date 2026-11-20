// Copyright (c) 2024 Raspberry Pi (Trading) Ltd.

// Generate DVI output using the command expander and TMDS encoder in HSTX.

// This example requires an external digital video connector connected to
// GPIOs 12 through 19 (the HSTX-capable GPIOs) with appropriate
// current-limiting resistors, e.g. 270 ohms. The pinout used in this example
// matches the Pico DVI Sock board, which can be soldered onto a Pico 2:
// https://github.com/Wren6991/Pico-DVI-Sock
//
// this is straight from HSTX examples, but with a hook for
// "frame start"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/hstx_ctrl.h"
#include "hardware/structs/hstx_fifo.h"
#include "hardware/structs/sio.h"
#include "pico/multicore.h"
#include "pico/sem.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "font_Arial_round_16x24.h"
#include "font_BigFont_16x16.h"
#include "font_Grotesk16x32.h"
#include "font_Tiny8.h"
#include "font_arial_bold.h"
#include "font_rom_237.h"

//
char current_draw_buffer[307200] __attribute__((aligned(16)));
#define framebuf current_draw_buffer

uint32_t sys_clock_actual;

// ----------------------------------------------------------------------------
// DVI constants

#define TMDS_CTRL_00 0x354u
#define TMDS_CTRL_01 0x0abu
#define TMDS_CTRL_10 0x154u
#define TMDS_CTRL_11 0x2abu

#define SYNC_V0_H0 (TMDS_CTRL_00 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V0_H1 (TMDS_CTRL_01 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H0 (TMDS_CTRL_10 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H1 (TMDS_CTRL_11 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))

#define MODE_H_SYNC_POLARITY 0
#define MODE_H_FRONT_PORCH 16
#define MODE_H_SYNC_WIDTH 96
#define MODE_H_BACK_PORCH 48
#define MODE_H_ACTIVE_PIXELS 640

#define MODE_V_SYNC_POLARITY 0
#define MODE_V_FRONT_PORCH 10
#define MODE_V_SYNC_WIDTH 2
#define MODE_V_BACK_PORCH 33
#define MODE_V_ACTIVE_LINES 480

#define MODE_H_TOTAL_PIXELS                                                    \
  (MODE_H_FRONT_PORCH + MODE_H_SYNC_WIDTH + MODE_H_BACK_PORCH +                \
   MODE_H_ACTIVE_PIXELS)
#define MODE_V_TOTAL_LINES                                                     \
  (MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + MODE_V_BACK_PORCH +                \
   MODE_V_ACTIVE_LINES)

#define HSTX_CMD_RAW (0x0u << 12)
#define HSTX_CMD_RAW_REPEAT (0x1u << 12)
#define HSTX_CMD_TMDS (0x2u << 12)
#define HSTX_CMD_TMDS_REPEAT (0x3u << 12)
#define HSTX_CMD_NOP (0xfu << 12)

// ----------------------------------------------------------------------------
// HSTX command lists

// Lists are padded with NOPs to be >= HSTX FIFO size, to avoid DMA rapidly
// pingponging and tripping up the IRQs.

static uint32_t vblank_line_vsync_off[] = {
    HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
    SYNC_V1_H1,
    HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
    SYNC_V1_H0,
    HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS),
    SYNC_V1_H1,
    HSTX_CMD_NOP};

static uint32_t vblank_line_vsync_on[] = {
    HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
    SYNC_V0_H1,
    HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
    SYNC_V0_H0,
    HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS),
    SYNC_V0_H1,
    HSTX_CMD_NOP};

static uint32_t vactive_line[] = {HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
                                  SYNC_V1_H1,
                                  HSTX_CMD_NOP,
                                  HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
                                  SYNC_V1_H0,
                                  HSTX_CMD_NOP,
                                  HSTX_CMD_RAW_REPEAT | MODE_H_BACK_PORCH,
                                  SYNC_V1_H1,
                                  HSTX_CMD_TMDS | MODE_H_ACTIVE_PIXELS};

// ----------------------------------------------------------------------------
// DMA logic

// FIX 1: Reserve channels through the SDK instead of assuming 0/1 are free.
static int DMACH_PING = -1;
static int DMACH_PONG = -1;
static bool dvi_initialized = false;

// First we ping. Then we pong. Then... we ping again.
static bool dma_pong = false;

// A ping and a pong are cued up initially, so the first time we enter this
// handler it is to cue up the second ping after the first ping has completed.
// This is the third scanline overall (-> =2 because zero-based).
static uint v_scanline = 2;

// During the vertical active period, we take two IRQs per scanline: one to
// post the command list, and another to post the pixels.
// every ~27 uSec,  we get two interrupts.
// The first is ~300 nSec, the second ~200 Nsec (clock is 150 MHz)
// time bewtween the pair is ~1.2 uSec
// so (0.3+0.2)/27 = 0.019, or about 2%
#define scr_start_line 500
#define hstx_clk 150000000
// stable 59 fps: sys_clk=250, hstx_clk=125, scr_start_liine=500
// stable 71 fps: sys_clk=300, hstx_clk=150, scr_start_liine=500
// stable 71 fps: sys_clk=450, hstx_clk=150, scr_start_liine=510
static bool vactive_cmdlist_posted = false;

int start_frame;

void __scratch_x("") dma_irq_handler() {
  // dma_pong indicates the channel that just finished, which is the one
  // we're about to reload.
  // gpio_put(2,1);
  uint ch_num = dma_pong ? DMACH_PONG : DMACH_PING;
  // FIX 2: Check completion status before advancing our
  // ping/pong state unless our expected channel has actually completed.
  if (!dma_channel_get_irq0_status(ch_num))
    return;
  dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
  dma_channel_acknowledge_irq0(ch_num);
  dma_pong = !dma_pong;

  if (v_scanline >= MODE_V_FRONT_PORCH &&
      v_scanline < (MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH)) {
    ch->read_addr = (uintptr_t)vblank_line_vsync_on;
    ch->transfer_count = count_of(vblank_line_vsync_on);
    // start_frame = true ;
  } else if (v_scanline <
             MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + MODE_V_BACK_PORCH) {
    ch->read_addr = (uintptr_t)vblank_line_vsync_off;
    ch->transfer_count = count_of(vblank_line_vsync_off);

    //
  } else if (!vactive_cmdlist_posted) {
    ch->read_addr = (uintptr_t)vactive_line;
    ch->transfer_count = count_of(vactive_line);
    vactive_cmdlist_posted = true;

    //
  } else {
    ch->read_addr = (uintptr_t)&framebuf[(v_scanline - (MODE_V_TOTAL_LINES -
                                                        MODE_V_ACTIVE_LINES)) *
                                         MODE_H_ACTIVE_PIXELS];
    ch->transfer_count = MODE_H_ACTIVE_PIXELS / sizeof(uint32_t);
    vactive_cmdlist_posted = false;
  }

  if (!vactive_cmdlist_posted) {
    // note that this staret time is critical because
    // screen erase takes most of vsynch time
    if (v_scanline == scr_start_line)
      start_frame = true; // 515
    // gpio_put(2,1);
    v_scanline = (v_scanline + 1) % MODE_V_TOTAL_LINES;
    // ;
  }
  // gpio_put(2,0);
}

// ----------------------------------------------------------------------------
// driver program

static __force_inline uint16_t colour_rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint16_t)r & 0xf8) >> 3 | ((uint16_t)g & 0xfc) << 3 |
         ((uint16_t)b & 0xf8) << 8;
}

static __force_inline uint8_t colour_rgb332(uint8_t r, uint8_t g, uint8_t b) {
  return (r & 0xc0) >> 6 | (g & 0xe0) >> 3 | (b & 0xe0) >> 0;
}

#define rgb332(r, g, b) ((((r) & 7) << 5) | (((g) & 7) << 2) | ((b) & 3))

// void scroll_framebuffer(void);

void DVIinit(void) {
  // FIX 3: Initialize once, on the core that will service the DVI IRQ.
  // DVI owns DMA_IRQ_0 exclusively; other users must use another DMA IRQ.
  if (dvi_initialized)
    panic("DVIinit called more than once");
  if (irq_get_exclusive_handler(DMA_IRQ_0) != NULL ||
      irq_has_shared_handler(DMA_IRQ_0))
    panic("DVI needs exclusive DMA_IRQ_0; a handler already exists");

  DMACH_PING = dma_claim_unused_channel(true);
  DMACH_PONG = dma_claim_unused_channel(true);
  dvi_initialized = true;

  // clk 125 MHz  ~59 Hz
  // clk 150 MHz 72 Hz
  clock_configure(
      clk_hstx,
      0,                                         // No aux mux
      CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLK_SYS, // Source from clk_sys
      sys_clock_actual, // 300000000, // Input frequency
      hstx_clk // 150000000  // Target HSTX clock frequency (divides by 2)
  );
  // Configure HSTX's TMDS encoder for RGB332
  hstx_ctrl_hw->expand_tmds = 2 << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB |
                              0 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB |
                              2 << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB |
                              29 << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB |
                              1 << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB |
                              26 << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;

  // Pixels (TMDS) come in 4 8-bit chunks. Control symbols (RAW) are an
  // entire 32-bit word.
  hstx_ctrl_hw->expand_shift = 4 << HSTX_CTRL_EXPAND_SHIFT_ENC_N_SHIFTS_LSB |
                               8 << HSTX_CTRL_EXPAND_SHIFT_ENC_SHIFT_LSB |
                               1 << HSTX_CTRL_EXPAND_SHIFT_RAW_N_SHIFTS_LSB |
                               0 << HSTX_CTRL_EXPAND_SHIFT_RAW_SHIFT_LSB;

  // Serial output config: clock period of 5 cycles, pop from command
  // expander every 5 cycles, shift the output shiftreg by 2 every cycle.
  hstx_ctrl_hw->csr = 0;
  hstx_ctrl_hw->csr = HSTX_CTRL_CSR_EXPAND_EN_BITS |
                      5u << HSTX_CTRL_CSR_CLKDIV_LSB |
                      5u << HSTX_CTRL_CSR_N_SHIFTS_LSB |
                      2u << HSTX_CTRL_CSR_SHIFT_LSB | HSTX_CTRL_CSR_EN_BITS;

  // Note we are leaving the HSTX clock at the SDK default of 125 MHz; since
  // we shift out two bits per HSTX clock cycle, this gives us an output of
  // 250 Mbps, which is very close to the bit clock for 480p 60Hz (252 MHz).
  // If we want the exact rate then we'll have to reconfigure PLLs.

  // HSTX outputs 0 through 7 appear on GPIO 12 through 19.
  // Pinout on Pico DVI sock:
  //
  //   GP12 D0+  GP13 D0-
  //   GP14 CK+  GP15 CK-
  //   GP16 D2+  GP17 D2-
  //   GP18 D1+  GP19 D1-

  // Assign clock pair to two neighbouring pins:
  hstx_ctrl_hw->bit[2] = HSTX_CTRL_BIT0_CLK_BITS;
  hstx_ctrl_hw->bit[3] = HSTX_CTRL_BIT0_CLK_BITS | HSTX_CTRL_BIT0_INV_BITS;
  for (uint lane = 0; lane < 3; ++lane) {
    // For each TMDS lane, assign it to the correct GPIO pair based on the
    // desired pinout:
    static const int lane_to_output_bit[3] = {0, 6, 4};
    int bit = lane_to_output_bit[lane];
    // Output even bits during first half of each HSTX cycle, and odd bits
    // during second half. The shifter advances by two bits each cycle.
    uint32_t lane_data_sel_bits = (lane * 10) << HSTX_CTRL_BIT0_SEL_P_LSB |
                                  (lane * 10 + 1) << HSTX_CTRL_BIT0_SEL_N_LSB;
    // The two halves of each pair get identical data, but one pin is inverted.
    hstx_ctrl_hw->bit[bit] = lane_data_sel_bits;
    hstx_ctrl_hw->bit[bit + 1] = lane_data_sel_bits | HSTX_CTRL_BIT0_INV_BITS;
  }

  for (int i = 12; i <= 19; ++i) {
    gpio_set_function(i, 0); // HSTX
  }

  // Both channels are set up identically, to transfer a whole scanline and
  // then chain to the opposite channel. Each time a channel finishes, we
  // reconfigure the one that just finished, meanwhile the opposite channel
  // is already making progress.
  dma_channel_config c;
  c = dma_channel_get_default_config(DMACH_PING);
  channel_config_set_chain_to(&c, DMACH_PONG);
  channel_config_set_dreq(&c, DREQ_HSTX);
  dma_channel_configure(DMACH_PING, &c, &hstx_fifo_hw->fifo,
                        vblank_line_vsync_off, count_of(vblank_line_vsync_off),
                        false);
  c = dma_channel_get_default_config(DMACH_PONG);
  channel_config_set_chain_to(&c, DMACH_PING);
  channel_config_set_dreq(&c, DREQ_HSTX);
  dma_channel_configure(DMACH_PONG, &c, &hstx_fifo_hw->fifo,
                        vblank_line_vsync_off, count_of(vblank_line_vsync_off),
                        false);

  // FIX 4: Install the exclusive DVI handler; enable only our channels.
  // Other peripherals must not route their completions to DMA_IRQ_0.
  // Register before enabling our sources; acknowledge only our channels.
  dma_channel_acknowledge_irq0(DMACH_PING);
  dma_channel_acknowledge_irq0(DMACH_PONG);
  irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
  dma_channel_set_irq0_enabled(DMACH_PING, true);
  dma_channel_set_irq0_enabled(DMACH_PONG, true);
  irq_set_enabled(DMA_IRQ_0, true);

  bus_ctrl_hw->priority =
      BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

  dma_channel_start(DMACH_PING);

  // while (1)
  //     //__wfi();
  //     for(int i=0; i<256; i++){
  //         Current_draw_buffer[10*640+i] = i ;
  //         sleep_ms(5) ;
  //     }
  //     memset(Current_draw_buffer, 0, 307200) ;
}
//==============================================================
// Drawing routines
//==============================================================
// Draw stuff
// A function for drawing a pixel with a specified color.
// Note that because information is passed to the PIO state machines through
// a DMA channel, we only need to modify the contents of the array and the
// pixels will be automatically updated on the screen.
void drawPixel(short x, short y, char color) {
  // Range checks (640x480 display)
  if ((x > 639) | (x < 0) | (y > 479) | (y < 0))
    return;

  // Which pixel is it?
  // shift by one to get the byte (two pixels/byte)
  // int pixel = (640 * y + x) >> 1;
  char *draw_loc = (current_draw_buffer + ((640 * y + x)));
  // draws to the current_draw_buffer
  *(draw_loc) = color;
}

// vertical line
void drawVLine(short x, short y, short h, char color) {
  for (short i = y; i < (y + h); i++) {
    drawPixel(x, i, color);
  }
}

// horizontal line
// note that this function draws using drawPiexl AND
// directly hitting the buffer memory for speed
void drawHLine(int x, int y, int w, char color) {
  // range checks
  if ((x >= 640) || (y >= 480))
    return;
  if ((x + w - 1) >= 640)
    w = 640 - x - 1;
  if (w < 1)
    return;
  //
  if (w == 1) {
    drawPixel(x, y, color);
    return;
  }
  //
  // loner pixel at x -- align left with next byte boundary
  //   if((x & 1)) {
  //     drawPixel(x,y,color);
  //     x++ ;
  //     w-- ;
  //   }
  // draw loner pixel at end and adjust width
  if ((w & 1)) {
    drawPixel(x + w - 1, y, color);
    w--;
  }
  // draw rest of line
  int len = (w);
  if (len > 0) //&& len+x < 640 && y<480
    memset(current_draw_buffer + (640 * y + (x)), color, len);
}

// general line drawing
// Bresenham's algorithm - thx wikipedia and thx Bruce!
// For drawLine
#define swap(a, b)                                                             \
  {                                                                            \
    short t = a;                                                               \
    a = b;                                                                     \
    b = t;                                                                     \
  }
void drawLine(short x0, short y0, short x1, short y1, char color) {
  /* Draw a straight line from (x0,y0) to (x1,y1) with given color
   * Parameters:
   *      x0: x-coordinate of starting point of line. The x-coordinate of
   *          the top-left of the screen is 0. It increases to the right.
   *      y0: y-coordinate of starting point of line. The y-coordinate of
   *          the top-left of the screen is 0. It increases to the bottom.
   *      x1: x-coordinate of ending point of line. The x-coordinate of
   *          the top-left of the screen is 0. It increases to the right.
   *      y1: y-coordinate of ending point of line. The y-coordinate of
   *          the top-left of the screen is 0. It increases to the bottom.
   *      color: 3-bit color value for line
   */
  short steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  short dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  short err = dx / 2;
  short ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0 <= x1; x0++) {
    if (steep) {
      drawPixel(y0, x0, color);
    } else {
      drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

// Draw a rectangle
void drawRect(short x, short y, short w, short h, char color) {
  /* Draw a rectangle outline with top left vertex (x,y), width w
   * and height h at given color
   * Parameters:
   *      x:  x-coordinate of top-left vertex. The x-coordinate of
   *          the top-left of the screen is 0. It increases to the right.
   *      y:  y-coordinate of top-left vertex. The y-coordinate of
   *          the top-left of the screen is 0. It increases to the bottom.
   *      w:  width of the rectangle
   *      h:  height of the rectangle
   *      color:  16-bit color of the rectangle outline
   * Returns: Nothing
   */
  drawHLine(x, y, w, color);
  drawHLine(x, y + h - 1, w, color);
  drawVLine(x, y, h, color);
  drawVLine(x + w - 1, y, h, color);
}

// fill a rectangle
void fillRect(short x, short y, short w, short h, char color) {
  /* Draw a filled rectangle with starting top-left vertex (x,y),
   *  width w and height h with given color
   * Parameters:
   *      x:  x-coordinate of top-left vertex; top left of screen is x=0
   *              and x increases to the right
   *      y:  y-coordinate of top-left vertex; top left of screen is y=0
   *              and y increases to the bottom
   *      w:  width of rectangle
   *      h:  height of rectangle
   *      color:  3-bit color value
   * Returns:     Nothing
   */
  if ((y + h - 1) >= 480)
    h = 480 - y - 1;

  for (int j = y; j < (y + h); j++) {
    drawHLine(x, j, w, color);
  }
}

void drawCircle(short x0, short y0, short r, char color) {
  /* Draw a circle outline with center (x0,y0) and radius r, with given color
   * Parameters:
   *      x0: x-coordinate of center of circle. The top-left of the screen
   *          has x-coordinate 0 and increases to the right
   *      y0: y-coordinate of center of circle. The top-left of the screen
   *          has y-coordinate 0 and increases to the bottom
   *      r:  radius of circle
   *      color: 16-bit color value for the circle. Note that the circle
   *          isn't filled. So, this is the color of the outline of the circle
   * Returns: Nothing
   */
  short f = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x = 0;
  short y = r;

  drawPixel(x0, y0 + r, color);
  drawPixel(x0, y0 - r, color);
  drawPixel(x0 + r, y0, color);
  drawPixel(x0 - r, y0, color);

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 + x, y0 - y, color);
    drawPixel(x0 - x, y0 - y, color);
    drawPixel(x0 + y, y0 + x, color);
    drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
  }
}

void drawCircleHelper(short x0, short y0, short r, unsigned char cornername,
                      char color) {
  // Helper function for drawing circles and circular objects
  short f = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x = 0;
  short y = r;

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
    if (cornername & 0x4) {
      drawPixel(x0 + x, y0 + y, color);
      drawPixel(x0 + y, y0 + x, color);
    }
    if (cornername & 0x2) {
      drawPixel(x0 + x, y0 - y, color);
      drawPixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      drawPixel(x0 - y, y0 + x, color);
      drawPixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      drawPixel(x0 - y, y0 - x, color);
      drawPixel(x0 - x, y0 - y, color);
    }
  }
}

// ==================================================
// int sqrt from https://github.com/chmike/fpsqrt/blob/master/fpsqrt.c
int32_t sqrt_i32(int32_t v) {
  uint32_t b = 1 << 30, q = 0, r = v;
  while (b > r)
    b >>= 2;
  while (b > 0) {
    uint32_t t = q + b;
    q >>= 1;
    if (r >= t) {
      r -= t;
      q += b;
    }
    b >>= 2;
  }
  return q;
}
// =============================
// fill a circle
// uses simple (but fast) sqrt algorithm
void fillCircle(short x0, short y0, short r, char color) {
  // adding r here just makes a better fit
  int r2 = r * r + r;
  if ((y0 - r < 0) || (y0 + r > 479))
    return;
  if ((x0 - r < 0) || (x0 + r > 639))
    return;
  for (int i = 0; i <= r; i++) {
    //
    int dx = sqrt_i32(r2 - i * i);
    // drawHLine(int x, int y, int w, char color)
    drawHLine(x0 - dx, y0 + (i), 2 * dx, color);
    drawHLine(x0 - dx, y0 - (i), 2 * dx, color);
  }
}
// ==================================================
// depricated
// void fillCircle(short x0, short y0, short r, char color) {
// /* Draw a filled circle with center (x0,y0) and radius r, with given color
//  * Parameters:
//  *      x0: x-coordinate of center of circle. The top-left of the screen
//  *          has x-coordinate 0 and increases to the right
//  *      y0: y-coordinate of center of circle. The top-left of the screen
//  *          has y-coordinate 0 and increases to the bottom
//  *      r:  radius of circle
//  *      color: 16-bit color value for the circle
//  * Returns: Nothing
//  */

//   drawVLine(x0, y0-r, 2*r+1, color);
//   fillCircleHelper(x0, y0, r, 3, 0, color);
// }

void fillCircleHelper(short x0, short y0, short r, unsigned char cornername,
                      short delta, char color) {
  // Helper function for drawing filled circles
  short f = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x = 0;
  short y = r;

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    if (cornername & 0x1) {
      drawVLine(x0 + x, y0 - y, 2 * y + 1 + delta, color);
      drawVLine(x0 + y, y0 - x, 2 * x + 1 + delta, color);
    }
    if (cornername & 0x2) {
      drawVLine(x0 - x, y0 - y, 2 * y + 1 + delta, color);
      drawVLine(x0 - y, y0 - x, 2 * x + 1 + delta, color);
    }
  }
}

// Draw a rounded rectangle
void drawRoundRect(short x, short y, short w, short h, short r, char color) {
  /* Draw a rounded rectangle outline with top left vertex (x,y), width w,
   * height h and radius of curvature r at given color
   * Parameters:
   *      x:  x-coordinate of top-left vertex. The x-coordinate of
   *          the top-left of the screen is 0. It increases to the right.
   *      y:  y-coordinate of top-left vertex. The y-coordinate of
   *          the top-left of the screen is 0. It increases to the bottom.
   *      w:  width of the rectangle
   *      h:  height of the rectangle
   *      color:  16-bit color of the rectangle outline
   * Returns: Nothing
   */
  // smarter version
  drawHLine(x + r, y, w - 2 * r, color);         // Top
  drawHLine(x + r, y + h - 1, w - 2 * r, color); // Bottom
  drawVLine(x, y + r, h - 2 * r, color);         // Left
  drawVLine(x + w - 1, y + r, h - 2 * r, color); // Right
  // draw four corners
  drawCircleHelper(x + r, y + r, r, 1, color);
  drawCircleHelper(x + w - r - 1, y + r, r, 2, color);
  drawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4, color);
  drawCircleHelper(x + r, y + h - r - 1, r, 8, color);
}

// =================================================
void fillRoundRect(short x, short y, short w, short h, short r, char color) {
  // faster version
  fillRect(x, y + r, w, h - 2 * r, color);
  fillRect(x + r, y, w - 2 * r, r, color);
  fillRect(x + r, y + h - r, w - 2 * r, r, color);

  // draw four corners
  fillCircle(x + w - r, y + r, r - 1, color);
  fillCircle(x + r, y + r, r - 1, color);
  fillCircle(x + w - r, y + h - r, r - 1, color);
  fillCircle(x + r, y + h - r, r - 1, color);
}

// // =================================================
// // Fill a rounded rectangle
// void fillRoundRect(short x, short y, short w, short h, short r, char color) {
//   // smarter version
//   fillRect(x+r, y, w-2*r, h, color);

//   // draw four corners
//   fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
//   fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
// }

/////////////////////////////////////////////////////////////////////
// copied with minor mods from
// https://ece4760.github.io/Projects/Fall2023/av522_dy245/code.html
/////////////////////////////////////////////////////////////////////
// Draw a filled triangle
// uses top-right rasterization rule to leave no holes between triangles
// (see https://en.wikipedia.org/wiki/Rasterisation)
void fillTri(float x0, float y0, float x1, float y1, float x2, float y2,
             char color) {
  //
  // sort verts so y0 <= y1 <= y2 (p0 = top, p1 = middle, p2 = bottom)
  if (y1 < y0) {
    swap(x0, x1);
    swap(y0, y1);
  }
  if (y2 < y0) {
    swap(x0, x2);
    swap(y0, y2);
  }
  if (y2 < y1) {
    swap(x1, x2);
    swap(y1, y2);
  }

  // calculate slopes of each edge, in fix15 (don't divide by 0)
  float dxdy_01 = y1 == y0 ? 0 : (x1 - x0) / (y1 - y0);
  float dxdy_02 = y2 == y0 ? 0 : (x2 - x0) / (y2 - y0);
  float dxdy_12 = y2 == y1 ? 0 : (x2 - x1) / (y2 - y1);
  // same for z
  // s15x16 dzdy_01 = y1 == y0 ? 0 : divs15x16(int_to_s15x16(z1 - z0), y1 - y0);
  // s15x16 dzdy_02 = y2 == y0 ? 0 : divs15x16(int_to_s15x16(z2 - z0), y2 - y0);
  // s15x16 dzdy_12 = y2 == y1 ? 0 : divs15x16(int_to_s15x16(z2 - z1), y2 - y1);

  // figure out whether p1 is on the left or right side of the triangle
  bool flat_top = (y0 == y1);
  bool flat_bottom = (y1 == y2);
  bool p1_is_left = flat_top ? (x0 > x1) : (dxdy_02 > dxdy_01);

  // starting at p0, we draw horizontal scanlines (from x_left to x_right, at
  // height y)
  float x_left = x0;
  float x_right = x0;
  float y = y0;

  // similarly, we have interpolators for z coordinates
  // s15x16 z_left = int_to_s15x16(z0) + zeropt5;
  // s15x16 z_right = int_to_s15x16(z0) + zeropt5;

  // x_left and x_right are moved based on slopes of left/right edges
  float dx_left, dx_right;
  float dz_left, dz_right;
  if (p1_is_left) {
    dx_left = dxdy_01;
    dx_right = dxdy_02;
    // dz_left = dzdy_01;
    // dz_right = dzdy_02;
  } else {
    dx_left = dxdy_02;
    dx_right = dxdy_01;
    // dz_left = dzdy_02;
    // dz_right = dzdy_01;
  }

// macro function to move the scanline down, and update its endpoints
#define moveScanline()                                                         \
  {                                                                            \
    y += 1;                                                                    \
    x_left += dx_left;                                                         \
    x_right += dx_right;                                                       \
  }
  // z_left += dz_left;\
  // z_right += dz_right;\
  }

  // draw top half of triangle; skipped for flat top case
  while (y < y1) {
    // drawScanline((short)fix2int15(y), (short)fix2int15(x_left),
    // (short)fix2int15(x_right), z_left, z_right, color);
    drawHLine((int)(x_left), (int)(y), abs((int)(x_right - x_left)), color);

    moveScanline();
  }

  // flat bottom triangles skip the rest
  if (flat_bottom)
    return;

  // reconfigure one end of the scanline so it goes from p1 to p2
  if (p1_is_left) {
    x_left = x1;
    dx_left = dxdy_12;
    // z_left = int2fix15(z1) + zeropt5;
    // dz_left = dzdy_12;
  } else {
    x_right = x1;
    dx_right = dxdy_12;
    // z_right = int2fix15(z1) + zeropt5;
    // dz_right = dzdy_12;
  }

  // draw horizontal line through p1 (middle)
  // drawScanline((short)fix2int15(y), (short)fix2int15(x_left),
  // (short)fix2int15(x_right), z_left, z_right, color);
  drawHLine((int)(x_left), (int)(y), (int)(x_right - x_left), color);
  // draw bottom half of triangle; skipped for flat bottom case
  while (y < y2) {
    moveScanline();
    // drawScanline((short)fix2int15(y), (short)fix2int15(x_left),
    // (short)fix2int15(x_right), z_left, z_right, color);
    drawHLine((int)(x_left), (int)(y), (int)(x_right - x_left), color);
  }
}

// =============================================
// end copied code
// =============================================
// application builds an array of
// short point_list[numlines][2]

// multiline draw
void drawMultiLine(int num_lines, short point_list[][2], char color) {
  for (int i = 1; i < num_lines; i++) {
    drawLine(point_list[i - 1][0], point_list[i - 1][1], point_list[i][0],
             point_list[i][1], color);
  }
}

//////////////
// now text
/////////////
//
// TinyFont from http://www.rinkydinkelectronics.com/r_fonts.php
//
#define getbit(in, bit) ((in >> bit) & 0x01)
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

int drawTextTiny8(short x, short y, char *str, char color, char bgcolor) {
  // get string start
  int char_count = 0;
  char *draw_loc = (current_draw_buffer + ((640 * y + x)));
  // error check
  if (x < 0 | y < 0 | y > 470)
    return 0;

  // holds a line of the char bit map
  unsigned char line;
  while (*str) {
    if ((x + 8 > 639))
      return char_count;
    // subtract 32 because first file wentry is <space>
    char c = (*str++) - 32;
    char_count++;
    for (int i = 0; i < 8; i++) {
      line = pgm_read_byte(TinyFont + ((int)c * 8) + i);
      // each  pixels is one byte, so write 8 bytes
      // using the value of 'line' to index into the pixel table//
      *(draw_loc + i * 640) = getbit(line, 7) ? color : bgcolor;
      *(draw_loc + i * 640 + 1) = getbit(line, 6) ? color : bgcolor;
      *(draw_loc + i * 640 + 2) = getbit(line, 5) ? color : bgcolor;
      *(draw_loc + i * 640 + 3) = getbit(line, 4) ? color : bgcolor;
      *(draw_loc + i * 640 + 4) = getbit(line, 3) ? color : bgcolor;
      *(draw_loc + i * 640 + 5) = getbit(line, 2) ? color : bgcolor;
      *(draw_loc + i * 640 + 6) = getbit(line, 1) ? color : bgcolor;
      *(draw_loc + i * 640 + 7) = getbit(line, 0) ? color : bgcolor;
    }
    draw_loc += 8;
    x += 8;
  }
  return char_count;
}
//

//  VGA437 from Code Block 437 IBM font 1982
int drawTextVGA437(short x, short y, char *str, char color, char bgcolor) {
  int char_count = 0;
  // get string start
  char *draw_loc = (current_draw_buffer + ((640 * y + x)));
  // error check
  if (x < 0 | y < 0 | x > 630 | y > 463)
    return 0;
  // set up the possible values for any byte

  // holds a line of the char bit map
  unsigned char line;
  while (*str) {
    if ((x + 8 > 639))
      return char_count;
    char c = *str++;
    char_count++;
    for (int i = 0; i < 16; i++) {
      line = pgm_read_byte(bigFont + ((int)c * 16) + i);
      //
      // using the value of 'line' to index into the pixel table//
      *(draw_loc + i * 640) = getbit(line, 7) ? color : bgcolor;
      *(draw_loc + i * 640 + 1) = getbit(line, 6) ? color : bgcolor;
      *(draw_loc + i * 640 + 2) = getbit(line, 5) ? color : bgcolor;
      *(draw_loc + i * 640 + 3) = getbit(line, 4) ? color : bgcolor;
      *(draw_loc + i * 640 + 4) = getbit(line, 3) ? color : bgcolor;
      *(draw_loc + i * 640 + 5) = getbit(line, 2) ? color : bgcolor;
      *(draw_loc + i * 640 + 6) = getbit(line, 1) ? color : bgcolor;
      *(draw_loc + i * 640 + 7) = getbit(line, 0) ? color : bgcolor;
    }
    draw_loc += 8;
    x += 8;
  }
  return char_count;
}
//
//
// Arial_round_16x24  bypasses the general drawPixel becuase of the
// packed nature of the draw buffer access
// http://www.rinkydinkelectronics.com/r_fonts.php
int drawTextArial24(short x, short y, char *str, char color, char bgcolor) {
  int char_count = 0;
  // get string start
  char *draw_loc = (current_draw_buffer + ((640 * y + x)));
  // error check
  if (x < 0 | y < 0 | y > 455)
    return 0;
  // holds a line of the char bit map
  unsigned short line;
  while (*str) {
    if ((x + 16 > 639))
      return char_count;
    // font filer startsa at character <space>
    char c = (*str++) - 32;
    char_count++;
    for (int i = 0; i < 24; i++) {
      line = pgm_read_byte(Arial_round_16x24 + ((int)c * 48) + 2 * i);
      // each two pixels is one byte, so write 4 bytes
      // using the value of 'line' to index into the pixel table//
      *(draw_loc + i * 640) = getbit(line, 7) ? color : bgcolor;
      *(draw_loc + i * 640 + 1) = getbit(line, 6) ? color : bgcolor;
      *(draw_loc + i * 640 + 2) = getbit(line, 5) ? color : bgcolor;
      *(draw_loc + i * 640 + 3) = getbit(line, 4) ? color : bgcolor;
      *(draw_loc + i * 640 + 4) = getbit(line, 3) ? color : bgcolor;
      *(draw_loc + i * 640 + 5) = getbit(line, 2) ? color : bgcolor;
      *(draw_loc + i * 640 + 6) = getbit(line, 1) ? color : bgcolor;
      *(draw_loc + i * 640 + 7) = getbit(line, 0) ? color : bgcolor;
      line = pgm_read_byte(Arial_round_16x24 + ((int)c * 48) + 2 * i + 1);
      // each two pixels is one byte, so write 4 bytes
      // using the value of 'line' to index into the pixel table//
      *(draw_loc + i * 640 + 8) = getbit(line, 7) ? color : bgcolor;
      *(draw_loc + i * 640 + 9) = getbit(line, 6) ? color : bgcolor;
      *(draw_loc + i * 640 + 10) = getbit(line, 5) ? color : bgcolor;
      *(draw_loc + i * 640 + 11) = getbit(line, 4) ? color : bgcolor;
      *(draw_loc + i * 640 + 12) = getbit(line, 3) ? color : bgcolor;
      *(draw_loc + i * 640 + 13) = getbit(line, 2) ? color : bgcolor;
      *(draw_loc + i * 640 + 14) = getbit(line, 1) ? color : bgcolor;
      *(draw_loc + i * 640 + 15) = getbit(line, 0) ? color : bgcolor;
    }
    draw_loc += 16;
    x += 16;
  }
  return char_count;
}
//
// Grotesk16x32
// http://www.rinkydinkelectronics.com/r_fonts.php
int drawTextGrotesk32(short x, short y, char *str, char color, char bgcolor) {
  int char_count = 0;
  // get string start
  char *draw_loc = (current_draw_buffer + ((640 * y + x)));
  // error check
  if (x < 0 | y < 0 | y > 479 - 32)
    return 0; //(x+16*strlen(str)>639)
  // set up the possible values for any byte
  // holds a line of the char bit map
  unsigned short line;
  while (*str) {
    // font filer startsa at character <space>
    if ((x + 16 > 639))
      return char_count;
    char c = (*str++) - 32;
    char_count++;
    for (int i = 0; i < 31; i++) {
      line = pgm_read_byte(Grotesk16x32 + ((int)c * 64) + 2 * i);
      // using the value of 'line' to index into the pixel table//
      *(draw_loc + i * 640) = getbit(line, 7) ? color : bgcolor;
      *(draw_loc + i * 640 + 1) = getbit(line, 6) ? color : bgcolor;
      *(draw_loc + i * 640 + 2) = getbit(line, 5) ? color : bgcolor;
      *(draw_loc + i * 640 + 3) = getbit(line, 4) ? color : bgcolor;
      *(draw_loc + i * 640 + 4) = getbit(line, 3) ? color : bgcolor;
      *(draw_loc + i * 640 + 5) = getbit(line, 2) ? color : bgcolor;
      *(draw_loc + i * 640 + 6) = getbit(line, 1) ? color : bgcolor;
      *(draw_loc + i * 640 + 7) = getbit(line, 0) ? color : bgcolor;
      line = pgm_read_byte(Grotesk16x32 + ((int)c * 64) + 2 * i + 1);
      // each two pixels is one byte, so write 4 bytes
      // using the value of 'line' to index into the pixel table//
      *(draw_loc + i * 640 + 8) = getbit(line, 7) ? color : bgcolor;
      *(draw_loc + i * 640 + 9) = getbit(line, 6) ? color : bgcolor;
      *(draw_loc + i * 640 + 10) = getbit(line, 5) ? color : bgcolor;
      *(draw_loc + i * 640 + 11) = getbit(line, 4) ? color : bgcolor;
      *(draw_loc + i * 640 + 12) = getbit(line, 3) ? color : bgcolor;
      *(draw_loc + i * 640 + 13) = getbit(line, 2) ? color : bgcolor;
      *(draw_loc + i * 640 + 14) = getbit(line, 1) ? color : bgcolor;
      *(draw_loc + i * 640 + 15) = getbit(line, 0) ? color : bgcolor;
    }
    draw_loc += 16;
    x += 16;
  }
  return char_count;
}

//
// bigfont 16x16
// http://www.rinkydinkelectronics.com/r_fonts.php
int drawTextBigFont_16x16(short x, short y, char *str, char color,
                          char bgcolor) {
  int char_count = 0;
  // get string start
  char *draw_loc = (current_draw_buffer + ((640 * y + x)));
  // error check
  if (x < 0 | y < 0 | y > 479 - 16)
    return 0; //(x+16*strlen(str)>639)
  // set up the possible values for any byte
  // holds a line of the char bit map
  unsigned short line;
  while (*str) {
    // font filer startsa at character <space>
    if ((x + 16 > 639))
      return char_count;
    char c = (*str++) - 32;
    char_count++;
    for (int i = 0; i < 15; i++) {
      line = pgm_read_byte(BigFont + ((int)c * 32) + 2 * i);
      // using the value of 'line' to index into the pixel table//
      *(draw_loc + i * 640) = getbit(line, 7) ? color : bgcolor;
      *(draw_loc + i * 640 + 1) = getbit(line, 6) ? color : bgcolor;
      *(draw_loc + i * 640 + 2) = getbit(line, 5) ? color : bgcolor;
      *(draw_loc + i * 640 + 3) = getbit(line, 4) ? color : bgcolor;
      *(draw_loc + i * 640 + 4) = getbit(line, 3) ? color : bgcolor;
      *(draw_loc + i * 640 + 5) = getbit(line, 2) ? color : bgcolor;
      *(draw_loc + i * 640 + 6) = getbit(line, 1) ? color : bgcolor;
      *(draw_loc + i * 640 + 7) = getbit(line, 0) ? color : bgcolor;
      line = pgm_read_byte(BigFont + ((int)c * 32) + 2 * i + 1);
      // each two pixels is one byte, so write 4 bytes
      // using the value of 'line' to index into the pixel table//
      *(draw_loc + i * 640 + 8) = getbit(line, 7) ? color : bgcolor;
      *(draw_loc + i * 640 + 9) = getbit(line, 6) ? color : bgcolor;
      *(draw_loc + i * 640 + 10) = getbit(line, 5) ? color : bgcolor;
      *(draw_loc + i * 640 + 11) = getbit(line, 4) ? color : bgcolor;
      *(draw_loc + i * 640 + 12) = getbit(line, 3) ? color : bgcolor;
      *(draw_loc + i * 640 + 13) = getbit(line, 2) ? color : bgcolor;
      *(draw_loc + i * 640 + 14) = getbit(line, 1) ? color : bgcolor;
      *(draw_loc + i * 640 + 15) = getbit(line, 0) ? color : bgcolor;
    }
    draw_loc += 16;
    x += 16;
  }
  return char_count;
}

//
// arial_bold 16x16
// http://www.rinkydinkelectronics.com/r_fonts.php
int drawTextArialBold_16x16(short x, short y, char *str, char color,
                            char bgcolor) {
  int char_count = 0;
  // get string start
  char *draw_loc = (current_draw_buffer + ((640 * y + x)));
  // error check
  if (x < 0 | y < 0 | y > 479 - 16)
    return 0; //(x+16*strlen(str)>639)
  // set up the possible values for any byte
  // holds a line of the char bit map
  unsigned short line;
  while (*str) {
    // font filer startsa at character <space>
    if ((x + 16 > 639))
      return char_count;
    char c = (*str++) - 32;
    char_count++;
    for (int i = 0; i < 15; i++) {
      line = pgm_read_byte(arial_bold + ((int)c * 32) + 2 * i);
      // using the value of 'line' to index into the pixel table//
      *(draw_loc + i * 640) = getbit(line, 7) ? color : bgcolor;
      *(draw_loc + i * 640 + 1) = getbit(line, 6) ? color : bgcolor;
      *(draw_loc + i * 640 + 2) = getbit(line, 5) ? color : bgcolor;
      *(draw_loc + i * 640 + 3) = getbit(line, 4) ? color : bgcolor;
      *(draw_loc + i * 640 + 4) = getbit(line, 3) ? color : bgcolor;
      *(draw_loc + i * 640 + 5) = getbit(line, 2) ? color : bgcolor;
      *(draw_loc + i * 640 + 6) = getbit(line, 1) ? color : bgcolor;
      *(draw_loc + i * 640 + 7) = getbit(line, 0) ? color : bgcolor;
      line = pgm_read_byte(arial_bold + ((int)c * 32) + 2 * i + 1);
      // each two pixels is one byte, so write 4 bytes
      // using the value of 'line' to index into the pixel table//
      *(draw_loc + i * 640 + 8) = getbit(line, 7) ? color : bgcolor;
      *(draw_loc + i * 640 + 9) = getbit(line, 6) ? color : bgcolor;
      *(draw_loc + i * 640 + 10) = getbit(line, 5) ? color : bgcolor;
      *(draw_loc + i * 640 + 11) = getbit(line, 4) ? color : bgcolor;
      *(draw_loc + i * 640 + 12) = getbit(line, 3) ? color : bgcolor;
      *(draw_loc + i * 640 + 13) = getbit(line, 2) ? color : bgcolor;
      *(draw_loc + i * 640 + 14) = getbit(line, 1) ? color : bgcolor;
      *(draw_loc + i * 640 + 15) = getbit(line, 0) ? color : bgcolor;
    }
    draw_loc += 16;
    x += 16;
  }
  return char_count;
}

// /////////////////////////////////////////////
// Fast erase functions
// NOTE that there is NO RANGE check on these funcitons
// They will clobber memory if x,y falls outside
// the DVI display boundaries (0,0) to (640,480)
// #define start_frame_erase \

void clearRect(short x1, short y1, short x2, short y2, short c) {
  for (int i = y1; i < y2; i++) {
    memset(current_draw_buffer + 640 * i + (x1), c, (x2 - x1));
  };
}
//
void clearLowFrame(short top, short c) {
  memset((current_draw_buffer + 640 * top), c, (307200 - 640 * top));
}
// region from y1 to y2 with y1 < y2
void clearRegion(short y1, short y2, short c) {
  memset((current_draw_buffer + 640 * y1), c, (640 * (y2 - y1)));
}

// ==================================================
// === convert HSV to rgb value
// ==================================================
char hsv2rgb(float h, float s, float v) {
  float C, X, m, rp, gp, bp;
  unsigned char r, g, b;
  // hsv to rgb conversion from
  // http://www.rapidtables.com/convert/color/hsv-to-rgb.htm
  C = v * s;
  // X = C * (1 - abs((int)(h/60)%2 - 1));
  //  (h/60) mod 2  = (h/60 - (int)(h/60))
  X = C * (1.0 - fabsf(fmodf(h / 60.0, 2.0) - 1.));
  m = v - C;
  if ((0 <= h) && (h < 60)) {
    rp = C;
    gp = X;
    bp = 0;
  } else if ((60 <= h) && (h < 120)) {
    rp = X;
    gp = C;
    bp = 0;
  } else if ((120 <= h) && (h < 180)) {
    rp = 0;
    gp = C;
    bp = X;
  } else if ((180 <= h) && (h < 240)) {
    rp = 0;
    gp = X;
    bp = C;
  } else if ((240 <= h) && (h < 300)) {
    rp = X;
    gp = 0;
    bp = C;
  } else if ((300 <= h) && (h < 360)) {
    rp = C;
    gp = 0;
    bp = X;
  } else {
    rp = 0;
    gp = 0;
    bp = 0;
  }
  // scale to 8-bit rgb
  r = (unsigned char)((rp + m) * 7);
  g = (unsigned char)((gp + m) * 7);
  b = (unsigned char)((bp + m) * 3);
  //
  return rgb332(r, g, b);
}
