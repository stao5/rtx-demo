/**
 * Sean Tao (sjt222@cornell.edu)
 *
 * This demonstrates ray tracing, by demonstrating rendering of 3 spheres
 * with camera centered at origin
 *
 *
 * HARDWARE CONNECTIONS
   - GPIO 16 ---> VGA Hsync
   - GPIO 17 ---> VGA Vsync
   - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
   - GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
   - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
   - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
   - RP2040 GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "VGA/vga16_graphics_v3.h"

// Include hardware libraries
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

// Include Pico libraries
#include "pico/multicore.h"
#include "pico/stdlib.h"

// Include standard libraries
#include <math.h>
#include <stdio.h>
#include <string.h>

// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// -------------------------------------------------------------
//                    Macros and Custom Types
// -------------------------------------------------------------

// Point type - What the fuck is a fixed point?? we only use floats here.

typedef struct Point3_t {
  float x;
  float y;
  float z;
} Point3;

// Sphere

typedef struct Sphere_t {
  Point3 center;
  float radius;
  char color;
} Sphere;

// Solution to intersection

typedef struct Soln_t {
  float t1;
  float t2;
} Soln;

// Light type
typedef enum { LIGHT_AMBIENT, LIGHT_POINT, LIGHT_DIRECTIONAL } lightType;

typedef struct Light_t {
  lightType type;
  float intensity;
  Point3 vect;
} Light;

#define ORIGIN ((Point3){0.0f, 0.0f, 0.0f})
#define CAMERA_LOC ORIGIN

#define FRAME_W 640
#define FRAME_H 480

#define CENTER_X (FRAME_W / 2)
#define CENTER_Y (FRAME_H / 2)

#define VPORT_W 1.0f
#define VPORT_H 0.75f
#define CAMERA_TO_VIEWPORT 1.0f // yields a FOV of approximately 53 deg

#define NUM_SPHERES 3
#define NUM_LIGHTS 3

// -------------------------------------------------------------
//                           Globals
// -------------------------------------------------------------

Point3 camera = CAMERA_LOC;
Sphere spheres[NUM_SPHERES] = {{{0.0f, -1.0f, 3.0f}, 1.0f, CYAN},
                               {{2.0f, 0.0f, 4.0f}, 1.0f, YELLOW},
                               {{-2.0f, 0.0f, 4.0f}, 1.0f, GREEN}};

// Intensities should not add up to more than 1, so that spots are not
// overexposed
Light lights[3] = {{LIGHT_AMBIENT, 0.2f, {0.0f, 0.0f, 0.0f}},
                   {LIGHT_POINT, 0.6f, {2.0f, 1.0f, 0.0f}},
                   {LIGHT_DIRECTIONAL, 0.2f, {1.0f, 4.0f, 4.0f}}};

// -------------------------------------------------------------
//                          Helper Fcns
// -------------------------------------------------------------

// v1 . v2
float dot(Point3 *x1, Point3 *x2) {
  return x1->x * x2->x + x1->y * x2->y + x1->z * x2->z;
}

// v1 + v2
Point3 add_vec(Point3 *x1, Point3 *x2) {
  return (Point3){x1->x + x2->x, x1->y + x2->y, x1->z + x2->z};
}

// v1 - v2
Point3 sub_vec(Point3 *x1, Point3 *x2) {
  return (Point3){x1->x - x2->x, x1->y - x2->y, x1->z - x2->z};
}

Point3 scalar_mul(Point3 *x1, float scalar) {
  return (Point3){x1->x * scalar, x1->y * scalar, x1->z * scalar};
}

// Turn screen coordinates into viewport point vector relative to origin
// note, we asssume the viewport is centered in front of the camera, at the
// prescribed offset distance

// we eventually will want to add rotation support to the viewport

Point3 canvas_to_viewport(int x, int y) {

  float x_cord = (float)(x - CENTER_X) * VPORT_W / FRAME_W;
  float y_cord = (float)(CENTER_Y - y) * VPORT_H / FRAME_H;

  return (Point3){x_cord, y_cord, CAMERA_TO_VIEWPORT};
}

// a sphere is relative easy, can be reduced to quadratic equation
// eventually we may have more shapes
Soln intersect_ray_sphere(Point3 *cam_ptr, Point3 *scrn_ptr,
                          Sphere *sphere_ptr) {

  float r = sphere_ptr->radius;
  Point3 co = sub_vec(cam_ptr, &sphere_ptr->center);
  Point3 ov = sub_vec(scrn_ptr, cam_ptr);

  float a = dot(&ov, &ov);
  float b = 2 * dot(&co, &ov);
  float c = dot(&co, &co) - r * r;

  float discrim = b * b - 4 * a * c;

  if (discrim < 0) {
    return (Soln){INFINITY, INFINITY};
  }

  float s1 = (-b - sqrt(discrim)) / (2 * a);
  float s2 = (-b + sqrt(discrim)) / (2 * a);

  return (Soln){s1, s2};
}

// returns the intensity of the light
float compute_lighting(Point3 *pt, Point3 *normal) {
  float intensity = 0.0f;
  Point3 light_vec;

  for (int i = 0; i < NUM_LIGHTS; i++) {

    // if ambient, simply add the light intensity
    if (lights[i].type == LIGHT_AMBIENT) {
      intensity += lights[i].intensity;
    }
    // if point light, calculate vector from src to point
    else if (lights[i].type == LIGHT_POINT) {
      light_vec = sub_vec(&lights[i].vect, pt);
    }
    // Otherwise, do select parallel ray
    else {
      light_vec = lights[i].vect;
    }

    float n_dot_l = dot(normal, &light_vec);
    float len_l = dot(&light_vec, &light_vec);

    if (n_dot_l > 0) {
      intensity += lights[i].intensity * n_dot_l / len_l;
    }
  }
  return intensity;
}

char trace_ray(Point3 *cam_ptr, Point3 *scrn_ptr, float t_min, float t_max) {

  float closest_t = INFINITY;
  Sphere *closest_sphere = NULL;

  for (int i = 0; i < NUM_SPHERES; i++) {

    // calculate solutions to ray, sphere intersection
    Soln sols = intersect_ray_sphere(cam_ptr, scrn_ptr, &spheres[i]);
    //  only pick closest intersection to draw, think objects stacked in front
    //  of each other
    if (sols.t1 >= t_min && sols.t1 <= t_max && sols.t1 < closest_t) {
      closest_t = sols.t1;
      closest_sphere = &spheres[i];
    }
    if (sols.t2 >= t_min && sols.t2 <= t_max && sols.t2 < closest_t) {
      closest_t = sols.t2;
      closest_sphere = &spheres[i];
    }
  }

  if (closest_sphere == NULL) {
    return WHITE;
  } else {
    Point3 ov = sub_vec(scrn_ptr, cam_ptr);
    Point3 ov_scaled = scalar_mul(&ov, closest_t);
    Point3 sphere_pt = add_vec(cam_ptr, &ov_scaled);

    Point3 norm_unscaled = sub_vec(&sphere_pt, &closest_sphere->center);
    float norm_mag_scalar = 1 / dot(&norm_unscaled, &norm_unscaled);
    Point3 norm = scalar_mul(&norm_unscaled, norm_mag_scalar);

    float intensity = compute_lighting(&sphere_pt, &norm);

    // temporary hack for 4 bit color :(
    if (intensity <= 0.25) {
      return closest_sphere->color - 3;
    } else if (intensity <= 0.31) {
      return closest_sphere->color - 2;
    } else if (intensity <= 0.37) {
      return closest_sphere->color - 1;
    } else {
      return closest_sphere->color;
    }
  }
}

// -------------------------------------------------------------
//                          Protothreads
// -------------------------------------------------------------

// Animation on core 1
static PT_THREAD(protothread_draw(struct pt *pt)) {
  // Mark beginning of thread
  PT_BEGIN(pt);

  while (1) {
    for (int y = 0; y < FRAME_H; y++) {
      for (int x = 0; x < FRAME_W; x++) {
        Point3 vport_coords = canvas_to_viewport(x, y);
        char color = trace_ray(&camera, &vport_coords, 1, INFINITY);
        drawPixel(x, y, color);
      }
    }

  } // END WHILE(1)
  PT_END(pt);
} // animation thread

int main() {

  // Overclock
  set_sys_clock_khz(150000, true);

  // Initialize stdio
  stdio_init_all();

  // Initialize VGA
  initVGA();

  // add threads
  pt_add_thread(protothread_draw);

  // start scheduler
  pt_schedule_start;
}
