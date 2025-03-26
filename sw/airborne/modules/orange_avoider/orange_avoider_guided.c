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
 * A Region of Interest (ROI) is implemented to only process green objects in the middle part of the bottom camera image.
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

// Default ROI settings if not defined in airframe file - REMOVED as not functional

uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND
};

// define settings
float oag_color_count_frac = 0.95f;       // obstacle detection threshold as a fraction of total of ROI (2% green required)
float oag_max_speed = 0.5f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(15.f);  // heading change setpoint for avoidance [rad/s]

// Define fixed scan area for green detection (40x80=3200 pixels) as fallback
#ifndef OAG_FIXED_SCAN_AREA
#define OAG_FIXED_SCAN_AREA 3000
#endif
uint32_t oag_fixed_scan_area = OAG_FIXED_SCAN_AREA;  // Default scan area (can be changed via GCS)
uint32_t current_roi_area = OAG_FIXED_SCAN_AREA;     // Actual area calculated from ROI

// Settings required by system but not used in this module
float __attribute__((unused)) oag_roi_width = 0.0f;    // Unused ROI width setting (required by settings system)
float __attribute__((unused)) oag_roi_height = 0.0f;   // Unused ROI height setting (required by settings system)

// define and initialise global variables
enum navigation_state_t navigation_state = OBSTACLE_FOUND;   // current state in state machine
int32_t color_count = 0;                // green color count from color filter for obstacle detection
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int32_t camera_pixel_count = 0;         // total number of pixels in the camera image
int16_t last_centroid_y = 0;            // store the last y-coordinate of the centroid

const int16_t max_trajectory_confidence = 1;  // number of consecutive negative object detections to be sure we are obstacle free

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
    current_roi_area = oag_fixed_scan_area;  // Use default if not provided
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
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // Initialize ROI area
  current_roi_area = oag_fixed_scan_area;

  // Calculate total camera pixels for threshold calculation
  camera_pixel_count = bottom_camera.output_size.w * bottom_camera.output_size.h;

  // Print camera info and ROI info for debugging
  VERBOSE_PRINT("Bottom camera resolution: %dx%d\n", 
                bottom_camera.output_size.w, bottom_camera.output_size.h);
  VERBOSE_PRINT("Using initial scan area of %d pixels for green detection\n", current_roi_area);

  // bind our colorfilter callback to receive the color filter output
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void orange_avoider_guided_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = OBSTACLE_FOUND;
    return;
  }

  // compute current color thresholds - use current ROI area for detection
  int32_t color_count_threshold = oag_color_count_frac * current_roi_area;

  VERBOSE_PRINT("Green_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  VERBOSE_PRINT("ROI area: %d pixels\n", current_roi_area);

  // Calculate speed based directly on obstacle presence
  float speed_sp = color_count < color_count_threshold ? 0.0f : oag_max_speed;

  switch (navigation_state){
    case SAFE:
      if (color_count < color_count_threshold){
        // Obstacle detected - stop and transition to OBSTACLE_FOUND state
        guidance_h_set_body_vel(0, 0);
        navigation_state = OBSTACLE_FOUND;
        
        // Determine turning direction based on centroid y-coordinate
        if (last_centroid_y >= 0) {
          // Green detected in upper part of ROI - turn clockwise
          avoidance_heading_direction = -1.0f;
          VERBOSE_PRINT("Obstacle in upper ROI (y=%d), turning clockwise\n", last_centroid_y);
        } else {
          // Green detected in lower part of ROI - turn counter-clockwise
          avoidance_heading_direction = 1.0f;
          VERBOSE_PRINT("Obstacle in lower ROI (y=%d), turning counter-clockwise\n", last_centroid_y);
        }
      } else {
        // No obstacle - proceed forward
        guidance_h_set_body_vel(speed_sp, 0);
      }
      break;

    case OBSTACLE_FOUND:
      // Stop the drone
      guidance_h_set_body_vel(0, 0);
      
      // Continue rotating in the same direction that was determined when the obstacle was first detected
      // This prevents oscillations that could occur if we changed direction during the search
      guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

      // Check if we've found a safe heading (no obstacle detected)
      if (color_count >= color_count_threshold) {
        // Safe heading found - stop rotating and transition back to SAFE state
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
 * Sets the heading direction for avoidance (no longer random)
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Initial default direction is clockwise
  avoidance_heading_direction = 1.f;
  VERBOSE_PRINT("Initial avoidance direction: clockwise\n");
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
 * Handler function for updating fixed scan area from GCS
 */
void orange_avoider_guided_SetFixedScanArea(float val)
{
  oag_fixed_scan_area = (uint32_t)val;
  // Also update current_roi_area as it might be used immediately
  current_roi_area = oag_fixed_scan_area;
  VERBOSE_PRINT("Updated fixed scan area to %d pixels\n", oag_fixed_scan_area);
}

