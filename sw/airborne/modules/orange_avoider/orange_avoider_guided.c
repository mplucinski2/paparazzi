/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider_guided.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the guided mode of the autopilot.
 * The avoidance strategy is to simply count the total number of green pixels in the ROI. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the green obstacles but for detecting the floor. When the total amount
 * of floor color drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 * 
 * A Region of Interest (ROI) is implemented to only process green objects in the middle part of the bottom camera image.
 * ROI dimensions can be configured in the airframe file.
 */

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[green_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Default ROI settings if not defined in airframe file - REMOVED as not functional

uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA
};

// define settings
float oag_color_count_frac = 0.02f;       // obstacle detection threshold as a fraction of total of ROI (2% green required)
float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
float oag_max_speed = 0.5f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]
float oag_heading_change = RadOfDeg(135.f); // fixed heading change angle for avoidance

// Define fixed scan area for green detection (40x80=3200 pixels)
#ifndef OAG_FIXED_SCAN_AREA
#define OAG_FIXED_SCAN_AREA 3200
#endif

// Settings required by system but not used in this module
float __attribute__((unused)) oag_roi_width = 0.0f;    // Unused ROI width setting (required by settings system)
float __attribute__((unused)) oag_roi_height = 0.0f;   // Unused ROI height setting (required by settings system)

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t color_count = 0;                // green color count from color filter for obstacle detection
int32_t floor_count = 0;                // floor color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int32_t camera_pixel_count = 0;         // total number of pixels in the camera image

const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free

// This call back will be used to receive the color count from the green detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the green filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  // The color detector module already applied ROI filtering
  // so we can directly use the detection results
  color_count = quality;
  
  // Debug output
  VERBOSE_PRINT("Green detected: quality=%d, pos=(%d,%d)\n", quality, pixel_x, pixel_y);
}

#ifndef FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the floor filter
#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;
}

/*
 * Initialisation function
 */
void orange_avoider_guided_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // Calculate total camera pixels for threshold calculation
  camera_pixel_count = bottom_camera.output_size.w * bottom_camera.output_size.h;

  // Print camera info and ROI info for debugging
  VERBOSE_PRINT("Bottom camera resolution: %dx%d\n", 
                bottom_camera.output_size.w, bottom_camera.output_size.h);
  VERBOSE_PRINT("Using fixed scan area of %d pixels for green detection\n", OAG_FIXED_SCAN_AREA);

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void orange_avoider_guided_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    return;
  }

  // compute current color thresholds - use fixed scan area for obstacle detection
  int32_t color_count_threshold = oag_color_count_frac * OAG_FIXED_SCAN_AREA;
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  VERBOSE_PRINT("Green_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  VERBOSE_PRINT("Fixed scan area: %d pixels\n", OAG_FIXED_SCAN_AREA);
  VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);

  // Calculate speed based directly on obstacle presence (not confidence)
  float speed_sp = color_count < color_count_threshold ? 0.0f : oag_max_speed;

  switch (navigation_state){
    case SAFE:
      if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
        navigation_state = OUT_OF_BOUNDS;
      } else if (color_count < color_count_threshold){
        navigation_state = OBSTACLE_FOUND;
      } else {
        guidance_h_set_body_vel(speed_sp, 0);
      }
      break;

    case OBSTACLE_FOUND:
      // stop
      guidance_h_set_body_vel(0, 0);

      // Set fixed 135-degree heading change instead of random direction
      avoidance_heading_direction = 1.f;  // Use positive direction
      
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;

    case SEARCH_FOR_SAFE_HEADING:
      // Use fixed heading change of 135 degrees
      guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi + avoidance_heading_direction * oag_heading_change);

      // Direct threshold check instead of confidence
      if (color_count < color_count_threshold) {
        // Still seeing obstacle, keep turning
      } else {
        // No obstacle detected, go back to SAFE state
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = SAFE;
      }
      break;

    case OUT_OF_BOUNDS:
      // stop
      guidance_h_set_body_vel(0, 0);

      // start turn back into arena
      guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(15));

      navigation_state = REENTER_ARENA;

      break;
    case REENTER_ARENA:
      // force floor center to opposite side of turn to head back into arena
      if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
        // return to heading mode
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);

        // reset safe counter
        navigation_state = SAFE;
      }
      break;
    default:
      break;
  }
  return;
}

/*
 * Sets the heading direction for avoidance (no longer random)
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Always use the same direction (clockwise)
  avoidance_heading_direction = 1.f;
  VERBOSE_PRINT("Set avoidance direction to: clockwise\n");
  return false;
}

/*
 * Handler function for updating heading rate from GCS
 */
void orange_avoider_guided_SetHeadingRate(float val)
{
  oag_heading_rate = RadOfDeg(val);
}

