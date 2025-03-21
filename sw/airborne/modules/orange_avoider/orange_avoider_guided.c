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
 * The avoidance strategy is to detect horizontal edges using color detection from cv_detect_color_object.
 * When the edge_ratio falls below a certain threshold, we assume there is an obstacle and turn.
 *
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 */

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>
#include <stdbool.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[edge_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND
};

// define settings
float oag_edge_ratio_threshold = 0.05f;    // edge detection threshold - ratio of columns with edge
float oag_max_speed = 0.3f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]

// Settings required by system but not used in this module
float __attribute__((unused)) oag_roi_width = 0.0f;    // Unused ROI width setting (required by settings system)
float __attribute__((unused)) oag_roi_height = 0.0f;   // Unused ROI height setting (required by settings system)
float __attribute__((unused)) oag_color_count_frac = 0.0f; // Unused color count fraction (required by settings system)

// define and initialise global variables
enum navigation_state_t navigation_state = OBSTACLE_FOUND;   // current state in state machine
float edge_ratio = 1.0f;                  // ratio of columns with edge to total columns
float avoidance_heading_direction = 0;    // heading change direction for avoidance [rad/s]
int32_t color_count = 0;                  // color count from color filter for obstacle detection
int16_t roi_size = 0;                     // size of the ROI in pixels (calculated during init)

// This call back will be used to receive the color count from the detector
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                              int16_t pixel_x, int16_t pixel_y,
                              int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                              int32_t quality, int16_t __attribute__((unused)) extra)
{
  // Update the color count (number of detected pixels)
  color_count = quality;
  
  // Calculate edge ratio based on detected color count
  // When the edge detection is enabled in cv_detect_color_object, the "quality" value
  // represents the number of columns with edges rather than raw pixel count
  edge_ratio = (float)color_count / roi_size;
  
  // Debug output
  VERBOSE_PRINT("Edge detection: count=%d, pos=(%d,%d), edge_ratio=%f\n", 
               quality, pixel_x, pixel_y, edge_ratio);
}

/*
 * Initialisation function
 */
void orange_avoider_guided_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // Calculate ROI size based on configuration in cv_detect_color_object module
  // This is the number of columns in the ROI, since edge detection works per column
#ifdef COLOR_OBJECT_DETECTOR_ROI_X_MIN1
  roi_size = COLOR_OBJECT_DETECTOR_ROI_X_MAX1 - COLOR_OBJECT_DETECTOR_ROI_X_MIN1;
#else
  roi_size = 40; // Default if not defined
#endif

  // bind our colorfilter callback to receive the color filter output
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);

  VERBOSE_PRINT("Edge-based obstacle avoidance initialized. ROI columns: %d, threshold: %f\n", 
                roi_size, oag_edge_ratio_threshold);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void orange_avoider_guided_periodic(void)
{
  // Only run the module if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = OBSTACLE_FOUND;
    return;
  }

  VERBOSE_PRINT("Edge_ratio: %f threshold: %f state: %d\n", edge_ratio, oag_edge_ratio_threshold, navigation_state);

  // Calculate speed based on edge detection (more edges = safer path)
  float speed_sp = edge_ratio < oag_edge_ratio_threshold ? 0.0f : oag_max_speed;

  switch (navigation_state){
    case SAFE:
      if (edge_ratio < oag_edge_ratio_threshold){
        // Edge not detected consistently - stop and transition to OBSTACLE_FOUND state
        guidance_h_set_body_vel(0, 0);
        navigation_state = OBSTACLE_FOUND;
        VERBOSE_PRINT("SAFE->OBSTACLE_FOUND: edge_ratio %f below threshold %f\n", edge_ratio, oag_edge_ratio_threshold);
      } else {
        // No obstacle - proceed forward
        guidance_h_set_body_vel(speed_sp, 0);
      }
      break;

    case OBSTACLE_FOUND:
      // Stop the drone
      guidance_h_set_body_vel(0, 0);
      
      // Continuously rotate to search for a safe heading
      guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

      // Check if we've found a safe heading (edge detected)
      if (edge_ratio >= oag_edge_ratio_threshold) {
        // Safe heading found - stop rotating and transition back to SAFE state
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = SAFE;
        VERBOSE_PRINT("OBSTACLE_FOUND->SAFE: edge_ratio %f above threshold %f\n", edge_ratio, oag_edge_ratio_threshold);
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

/*
 * Handler function for updating color count fraction from GCS
 */
void orange_avoider_guided_SetColorCountFrac(float val)
{
  oag_color_count_frac = val;
  // This is not used directly since we're using edge detection
  VERBOSE_PRINT("Color count fraction set to %f (not used with edge detection)\n", val);
}

