/**
 * Sean Tao (sjt222@cornell.edu)
 *
 * This demonstrates ray tracing, by demonstrating rendering of 3 spheres
 * with camera centered at origin
 *
 *
 * HARDWARE CONNECTIONS
 *
 *
 * RESOURCES USED
 *
 *
 */

// Include the DVI grahics library developed by RPi and Bruce Land
// (bruce.land@cornell.edu)
#include "DVI/DVIinit_320x240x24.h"

// Include hardware libraries
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/vreg.h"

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

// Point type
typedef struct Point3_t {
  float x;
  float y;
  float z;
} Point3;

// Color type

typedef struct RGB_t {
  float color[3];
} RGB;

// Sphere

typedef struct Sphere_t {
  Point3 center;
  float radius;
  float color[3];
  int specular;
  float reflectivity;
} Sphere;

// Solution to intersection

typedef struct Soln_t {
  float t1;
  float t2;
} Soln;

typedef struct SphereTimePair_t {
  Sphere *sphere_ptr;
  float t;
} SphereTimePair;

// Light type
typedef enum { LIGHT_AMBIENT, LIGHT_POINT, LIGHT_DIRECTIONAL } lightType;

typedef struct Light_t {
  lightType type;
  float intensity;
  Point3 vect;
} Light;

#define DEBUG_GPIO 11

#define ORIGIN ((Point3){0.0f, 0.0f, 0.0f})
#define CAMERA_LOC ORIGIN

#define FRAME_W 320
#define FRAME_H 240

#define CENTER_X (FRAME_W / 2)
#define CENTER_Y (FRAME_H / 2)

#define VPORT_W 1.0f
#define VPORT_H 0.75f
#define CAMERA_TO_VIEWPORT 1.0f // yields a FOV of approximately 53 deg

#define NUM_SPHERES 4
#define NUM_LIGHTS 3

#define RED_POS 0
#define GREEN_POS 1
#define BLUE_POS 2

#define WHITE ((RGB){{1.0f, 1.0f, 1.0f}})
#define DARK ((RGB){{0.43f, 0.43f, 0.67f}})

#define REFLECTION_RECURSION 2 // 3 levels of mirroring

#define SURFACE_BIAS 0.01f // offset for shadow/reflection ray origins

// -------------------------------------------------------------
//                           Globals
// -------------------------------------------------------------

// NOTE, we're using 888 RGB, colors are 0.0-1.0 per channel

Point3 camera = CAMERA_LOC;
Sphere spheres[NUM_SPHERES] = {
    // FORMAT
    // Center{x, y, z}, radius, rgb{r, g, b}, specular factor (how shiny),
    // reflectivity (how mirror like)
    // RED
    {{0.0f, -1.0f, 3.0f}, 1.0f, {1.00f, 0.29f, 0.33f}, 10, 0.2},
    // GREEN
    {{2.0f, 0.0f, 4.0f}, 1.0f, {0.14f, 1.00f, 0.67f}, -1, 0.3},
    // PURPLE
    {{-2.0f, 0.0f, 4.0f}, 1.0f, {0.71f, 0.29f, 1.00f}, 500, 0.4},
    // YELLOW (BELOW ALL THE OTHERS)
    {{0.0f, -5001.0f, 0.0f}, 5000.0f, {1.00f, 1.00f, 0.00f}, 1000, 0.5}};

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

// clamp a 0.0-1.0 color channel and scale it to 0-255
uint8_t to_byte(float v) {
  if (v <= 0.0f)
    return 0;
  if (v >= 1.0f)
    return 255;
  return (uint8_t)(v * 255.0f + 0.5f);
}

// Turn screen coordinates into viewport point vector relative to origin
// note, we asssume the viewport is centered in front of the camera, at the
// prescribed offset distance

// we eventually will want to add rotation support to the viewport

Point3 canvas_to_viewport(int x, int y) {

  float x_cord = (float)(x - CENTER_X) * VPORT_W / FRAME_W;
  float y_cord = (float)(CENTER_Y - y) * VPORT_H / FRAME_H;

  return (Point3){camera.x + x_cord, camera.y + y_cord,
                  camera.z + CAMERA_TO_VIEWPORT};
}

// reflect an incoming ray helper about the normal
Point3 reflect_ray(Point3 *ray, Point3 *norm) {
  Point3 rescaled_light_vec = scalar_mul(norm, 2 * dot(norm, ray));
  Point3 reflect_vec = sub_vec(&rescaled_light_vec, ray);

  return reflect_vec;
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

  float s1 = (-b - sqrtf(discrim)) / (2 * a);
  float s2 = (-b + sqrtf(discrim)) / (2 * a);

  return (Soln){s1, s2};
}

// caclulates specular scaling effect with Schlick's Approximation (to avoid
// costly pow invocation)

float spec_approx(float x, int n) {
  float denom = (float)n - n * x + x;
  return x / denom;
}

SphereTimePair closest_intersection(Point3 *cam_ptr, Point3 *scrn_ptr,
                                    float t_min, float t_max) {

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

  SphereTimePair ret = {closest_sphere, closest_t};

  return ret;
}

// returns the intensity of the light
float compute_lighting(Point3 *pt, Point3 *normal, Point3 *to_cam,
                       int specular) {
  float intensity = 0.0f;
  Point3 light_vec;

  float t_max = -1;

  for (int i = 0; i < NUM_LIGHTS; i++) {

    // if ambient, simply add the light intensity
    if (lights[i].type == LIGHT_AMBIENT) {
      intensity += lights[i].intensity;
      continue;
    }
    // if point light, calculate vector from src to point
    else if (lights[i].type == LIGHT_POINT) {
      light_vec = sub_vec(&lights[i].vect, pt);
      t_max = 1;
    }
    // Otherwise, do select parallel ray
    else {
      light_vec = lights[i].vect;
      t_max = INFINITY;
    }

    // Shadow Check
    // closest_intersection takes a second point on the ray, not a direction
    Point3 shadow_pt = add_vec(pt, &light_vec);
    SphereTimePair shadow_pair =
        closest_intersection(pt, &shadow_pt, 0.001, t_max);

    Sphere *sphere_ptr = shadow_pair.sphere_ptr;
    // if there is a shadow, don't light
    if (sphere_ptr != NULL)
      continue;

    // diffuse lighting
    float n_dot_l = dot(normal, &light_vec);
    float len_l = sqrtf(dot(&light_vec, &light_vec));

    if (n_dot_l > 0) {
      intensity += lights[i].intensity * n_dot_l / len_l;
    }

    // specular lighting
    if (specular != -1) {

      Point3 reflect_vec = reflect_ray(&light_vec, normal);

      float r_dot_v = dot(&reflect_vec, to_cam);

      if (r_dot_v > 0) {
        float len_r = sqrtf(dot(&reflect_vec, &reflect_vec));
        float len_v = sqrtf(dot(to_cam, to_cam));

        float cos_alpha = r_dot_v / (len_r * len_v);

        intensity += lights[i].intensity * spec_approx(cos_alpha, specular);
      }
    }
  }
  return intensity;
}

RGB trace_ray(Point3 *cam_ptr, Point3 *scrn_ptr, float t_min, float t_max,
              int recursion_depth) {

  // ======================================
  // Compute local contribution from sphere
  // ======================================

  // find nearest intersection with the camera
  SphereTimePair closest =
      closest_intersection(cam_ptr, scrn_ptr, t_min, t_max);

  Sphere *closest_sphere = closest.sphere_ptr;
  float closest_t = closest.t;

  if (closest_sphere == NULL) {
    return DARK;
  } else {
    Point3 ov = sub_vec(scrn_ptr, cam_ptr);
    Point3 vo = scalar_mul(&ov, -1);

    Point3 ov_scaled = scalar_mul(&ov, closest_t);
    Point3 sphere_pt = add_vec(cam_ptr, &ov_scaled);

    Point3 norm_unscaled = sub_vec(&sphere_pt, &closest_sphere->center);
    float norm_mag_scalar = 1 / sqrtf(dot(&norm_unscaled, &norm_unscaled));
    Point3 norm = scalar_mul(&norm_unscaled, norm_mag_scalar);

    // start secondary rays slightly off the surface (avoids self-hits)
    Point3 bias = scalar_mul(&norm, SURFACE_BIAS);
    Point3 start_pt = add_vec(&sphere_pt, &bias);

    float intensity =
        compute_lighting(&start_pt, &norm, &vo, closest_sphere->specular);

    if (intensity > 1.0) {
      intensity = 1.0;
    }

    float local_red_comp = closest_sphere->color[RED_POS] * intensity;

    float local_green_comp = closest_sphere->color[GREEN_POS] * intensity;

    float local_blue_comp = closest_sphere->color[BLUE_POS] * intensity;

    // ======================================
    // Compute mirror effect contribution
    // ======================================

    float r = closest_sphere->reflectivity;

    if (recursion_depth <= 0 || r <= 0)
      return ((RGB){local_red_comp, local_green_comp, local_blue_comp});

    // Compute reflected color
    Point3 reflected_ray = reflect_ray(&vo, &norm);
    Point3 reflected_pt = add_vec(
        &reflected_ray,
        &start_pt); // trace ray will subtract start_pt upon functional call

    RGB reflected_color = trace_ray(&start_pt, &reflected_pt, 0.0001,
                                    INFINITY, recursion_depth - 1);

    float comb_red_comp =
        local_red_comp * (1 - r) + reflected_color.color[0] * r;

    float comb_green_comp =
        local_green_comp * (1 - r) + reflected_color.color[1] * r;

    float comb_blue_comp =
        local_blue_comp * (1 - r) + reflected_color.color[2] * r;

    return ((RGB){comb_red_comp, comb_green_comp, comb_blue_comp});
  }
}

// -------------------------------------------------------------
//                          Protothreads
// -------------------------------------------------------------

// draw on core 0 (top half of screen)
static PT_THREAD(protothread_draw(struct pt *pt)) {
  // Mark beginning of thread
  PT_BEGIN(pt);

  while (1) {
    for (int y = 0; y < FRAME_H / 2; y++) {
      for (int x = 0; x < FRAME_W; x++) {
        Point3 vport_coords = canvas_to_viewport(x, y);
        RGB split_color = trace_ray(&camera, &vport_coords, 1, INFINITY,
                                    REFLECTION_RECURSION);
        uint32_t color = RGB888(to_byte(split_color.color[0]),
                                to_byte(split_color.color[1]),
                                to_byte(split_color.color[2]));
        drawPixel(x, y, color);
      }
    }
    spheres[0].center.z += 0.01;
    spheres[0].center.y += 0.01;
    spheres[0].center.x -= 0.02;
    spheres[2].center.z += 0.05;
    spheres[2].center.x += 0.01;
  } // END WHILE(1)
  PT_END(pt);
} // draw thread (Core 0)

// draw on core 1 (bottom half of screen)
static PT_THREAD(protothread_draw1(struct pt *pt)) {
  // Mark beginning of thread
  PT_BEGIN(pt);

  while (1) {
    for (int y = FRAME_H / 2; y < FRAME_H; y++) {
      for (int x = 0; x < FRAME_W; x++) {
        Point3 vport_coords = canvas_to_viewport(x, y);
        RGB split_color = trace_ray(&camera, &vport_coords, 1, INFINITY,
                                    REFLECTION_RECURSION);
        uint32_t color = RGB888(to_byte(split_color.color[0]),
                                to_byte(split_color.color[1]),
                                to_byte(split_color.color[2]));
        drawPixel(x, y, color);
      }
    }
  } // END WHILE(1)
  PT_END(pt);
} // draw thread (Core 1)

// Core 1 Main
void core1_main() {
  // Add animation thread
  pt_add_thread(protothread_draw1);
  // Start the scheduler
  pt_schedule_start;
}

// Core 0 Main
int main() {

  // Overclock
  vreg_disable_voltage_limit();
  vreg_set_voltage(VREG_VOLTAGE_1_30);

  set_sys_clock_khz(300000, false);

  // stablize clock
  sleep_ms(10);

  // record clock for HSTX setup
  sys_clock_actual = clock_get_hz(clk_sys);

  // Initialize stdio
  stdio_init_all();

  // Initialize DVI
  DVIinit();

  // Initialize debug GPIO pin
  gpio_set_dir(DEBUG_GPIO, GPIO_IN);
  gpio_pull_up(DEBUG_GPIO);

  // Initialize second core
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_draw);

  // start scheduler
  pt_schedule_start;
}
