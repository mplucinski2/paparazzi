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
 * A Region of Interest (ROI) is implemented to only process green objects in the middle part of the camera image.
 * ROI dimensions can be configured in the airframe file.
 */

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[green_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Define fixed scan area fallback for detection
#define OAG_FIXED_SCAN_AREA 3200

// Function declaration for turning direction determination
void determineAvoidanceDirection(int16_t centroid_y);

// define and initialise global variables
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING
};

// define settings
float oag_color_count_frac = 0.78f;       // obstacle detection threshold as a fraction of total of ROI (2% green required)
float oag_max_speed = 0.4f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]

// Current calculated ROI area
uint32_t current_roi_area = OAG_FIXED_SCAN_AREA;     // Actual area calculated from ROI

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t color_count = 0;                // green color count from color filter for obstacle detection
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe
int16_t last_centroid_y = 0;            // store the last y-coordinate of the centroid

// Number of consecutive negative object detections to be sure we are obstacle free
int16_t max_trajectory_confidence = 5;

// This call back will be used to receive the color count from the green detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires a color filter, please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the obstacle filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t extra)
{
  // The color detector module already applied ROI filtering
  // so we can directly use the detection results
  color_count = quality;
  
  // Store the centroid y-coordinate for direction decision
  last_centroid_y = pixel_y;
  
  // Extract ROI area from extra field (if provided)
  if (extra > 0) {
    current_roi_area = (uint32_t)extra;
  } else {
    current_roi_area = OAG_FIXED_SCAN_AREA;  // Use default if not provided
  }
  
  // Debug output
  VERBOSE_PRINT("Green detected: quality=%d, pos=(%d,%d), ROI area=%d\n", 
                quality, pixel_x, pixel_y, current_roi_area);
}

/*
 * Initialisation function
 */
void orange_avoider_guided_init(void)
{
  // Initialise values
  srand(time(NULL));
  // Set initial direction (will be updated based on centroid)
  avoidance_heading_direction = 1.0f;

  // Print ROI info for debugging
  VERBOSE_PRINT("Using initial scan area of %d pixels for green detection\n", current_roi_area);

  // bind our colorfilter callback to receive the color filter output
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void orange_avoider_guided_periodic(void)
{
  // Only run the module if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return;
  }

  // compute current color thresholds - use current ROI area for obstacle detection
  int32_t color_count_threshold = oag_color_count_frac * current_roi_area;

  VERBOSE_PRINT("Green_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  VERBOSE_PRINT("ROI area: %d pixels\n", current_roi_area);

  // update our safe confidence using color threshold
  if(color_count > color_count_threshold){
    obstacle_free_confidence++;  // More green means safe
  } else {
    obstacle_free_confidence -= 2;  // Less green means obstacle detected
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float speed_sp = fminf(oag_max_speed, 0.2f * obstacle_free_confidence);

  switch (navigation_state){
    case SAFE:
      if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else {
        guidance_h_set_body_vel(speed_sp, 0);
      }
      break;

    case OBSTACLE_FOUND:
      // stop
      guidance_h_set_body_vel(0, 0);

      // determine avoidance direction based on centroid position
      determineAvoidanceDirection(last_centroid_y);

      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;

    case SEARCH_FOR_SAFE_HEADING:
      guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= max_trajectory_confidence){
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = SAFE;
      }
      break;
      
    default:
      break;
  }
  return;
}

/*
 * Determines turning direction based on centroid position
 */
void determineAvoidanceDirection(int16_t centroid_y)
{
  // Determine turning direction based on centroid y-coordinate
  if (centroid_y >= 0) {
    // Green detected in upper part of ROI - turn clockwise
    avoidance_heading_direction = -1.0f;
    VERBOSE_PRINT("Obstacle in upper ROI (y=%d), turning clockwise\n", centroid_y);
  } else {
    // Green detected in lower part of ROI - turn counter-clockwise
    avoidance_heading_direction = 1.0f;
    VERBOSE_PRINT("Obstacle in lower ROI (y=%d), turning counter-clockwise\n", centroid_y);
  }
}

/*
 * Handler function for updating heading rate from GCS
 */
void orange_avoider_guided_SetHeadingRate(float val)
{
  oag_heading_rate = RadOfDeg(val);
}

