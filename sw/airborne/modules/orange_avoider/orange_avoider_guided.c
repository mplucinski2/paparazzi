/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider_guided.c"
 * @author Kirk Scheper, Miłosz Pluciński (modification)
 * the code is a modification of the original orange_avoider_guided.c. It detects the green pixels in a cropped rectangle (region of interest) at the bottom of the front camera image,
 * it detects obstacle if the number of green (floor) pixels is below a chosen threshold (percentage). There is no need to distinguish floor and obstacles, both are detected with this logic (optitrack not used for out of bounds)
 * The cv_color_detection module was modified to use the region of interest parameters, and apart from that the same module is used as for the original avoider. 
 * Finally, now the centroid  functionality of the original module is used to determine if the centroid of the green pixels is on the left or right side of the image, making it possible to determine
 * if the obstacle is on the left or right side of the drone, which helps to more efficiently determine the direction of rotation when looking for safe heading (the direction is chosen once, when obstacle is detected
 * when the drone is turning the direction of rotation is kept constant to avoid oscillation. y coordinate is used, as the front camera feed is rotated 90 degrees.
 * overall this approach works if the obstacle is not floating in the air, and if the obstacle does not have exactly the same color as the floor.
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

//fallback total number of pixels in the region of interest (roi)
#define OAG_FIXED_SCAN_AREA 3200

//function declaration for determining the ccw or cw rotation when seeing an obstacle
void determineAvoidanceDirection(int16_t centroid_y);

//defining global variables
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING
};

//defining settngs
float oag_color_count_frac = 0.78f;       //threshold (if the fraction of green pixels is below this value, it is treated as obstacle detection)
float oag_max_speed = 0.4f;               //maximum flight speed in m/s
float oag_heading_rate = RadOfDeg(20.f);  //heading rate

//current roi area
uint32_t current_roi_area = OAG_FIXED_SCAN_AREA;  

//defining and initialising global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   //current state
int32_t color_count = 0;                //number of green pixels in the roi
float avoidance_heading_direction = 0;  //direction of rotations (ccw or cw in {-1, 1})
int16_t obstacle_free_confidence = 0;   //the same measure as used in the original file
int16_t last_centroid_y = 0;            //y coordinate of the centroid of the green pixels upon last obstacle detection

//how many "obstacle free" readings are needed to make sure that the heading is safe
int16_t max_trajectory_confidence = 5;

//callback to receive the color count from the cv_Color_detection
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
  //the module already utilizes roi, the number of green pixels directly taken from the module
  color_count = quality;
  
  //used for the direction of rotation estimation
  last_centroid_y = pixel_y;
  
  //using roi area if provided, otherwise using a fallback fixed value
  if (extra > 0) {
    current_roi_area = (uint32_t)extra;
  } else {
    current_roi_area = OAG_FIXED_SCAN_AREA; 
  }
  
  //debugging print
  VERBOSE_PRINT("green detected: quality=%d, pos=(%d,%d), ROI area=%d\n", 
                quality, pixel_x, pixel_y, current_roi_area);
}

/*
 *init function
 */
void orange_avoider_guided_init(void)
{
  // Initialise values
  srand(time(NULL));
  //initial rotation direction when changing heading (is determined after)
  avoidance_heading_direction = 1.0f;

  //printing roi info for debugging
  VERBOSE_PRINT("using initial scan area of %d pixels for green detection\n", current_roi_area);

  //receiving the color count from the cv_color_detection
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

/*
 *functions checking if it is safe to move forwards, analogous to the original orange avoider
 */
void orange_avoider_guided_periodic(void)
{
  //only running the function if we are in guided mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return;
  }

  //current number of green pixels threshold, calculated as a fraction times the total number of pixels in the region of interest
  int32_t color_count_threshold = oag_color_count_frac * current_roi_area;

  VERBOSE_PRINT("number of green pixels: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  VERBOSE_PRINT("region of interest area: %d pixels\n", current_roi_area);

  // update our safe confidence using color threshold
  if(color_count > color_count_threshold){
    obstacle_free_confidence++;  //number of green pixels above threshold, no obstacles
  } else {
    obstacle_free_confidence -= 2;  //too few green pixels detected, obstacle detected (at least 3 detections are needed (done because of noise in real flight) for the current max obbstacle_free_confidence of 5)
  }

  //obstacle free confidence, defined as in the original orange avoider
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
      //stop
      guidance_h_set_body_vel(0, 0);

      //determing rotation direction
      determineAvoidanceDirection(last_centroid_y);

      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;

    case SEARCH_FOR_SAFE_HEADING:
      guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

      //making sure that a few detections are "safe" before moving in a given heading
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
 *determing the direction of rotation based on the centroid of the green pixels
 */
void determineAvoidanceDirection(int16_t centroid_y)
{
  //using centroid of the green pixels to determine if the obstacle is on the left or right, then turning in the opposite direction, the direction is determined once when obstacle is detected
  //it is not changed when turning to avoid oscillations
  if (centroid_y >= 0) {
    //centroid of the green pixels is on the right > obstacle on the left > turning clockwise
    avoidance_heading_direction = -1.0f;
    VERBOSE_PRINT("obstacle in upper ROI (y=%d), turning clockwise\n", centroid_y);
  } else {
    //green centroid on the left > obstacle on the right > turning counter-clockwise
    avoidance_heading_direction = 1.0f;
    VERBOSE_PRINT("obstacle in lower ROI (y=%d), turning counter-clockwise\n", centroid_y);
  }
}

/*
 * handler function for updating the heading rate in the gcs
 */
void orange_avoider_guided_SetHeadingRate(float val)
{
  oag_heading_rate = RadOfDeg(val);
}

