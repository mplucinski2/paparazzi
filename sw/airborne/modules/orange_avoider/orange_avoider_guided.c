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
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the orange poles but for green to detect the floor. When the total amount
 * of green drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 */

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>


#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  BACKING_UP,       // New state for backward movement
  ROTATE_FIXED,     // New state for fixed rotation
  REENTER_ARENA
};

// define settings
float oag_color_count_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
float oag_floor_count_frac = 0.95f;       // floor detection threshold as a fraction of total of image
float oag_max_speed = 0.5f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.
// Add global variables for timing and angle tracking
float backward_timer = 0;
float rotation_timer = 0;
float fixed_rotation_angle = 0;
const float BACKUP_DURATION = 2.0f;  // seconds to back up
const float ROTATION_DURATION = 3.0f; // seconds to rotate
const float BACKUP_SPEED = 0.7f;     // m/s backward speed
const float FIXED_ROTATION_ANGLE = RadOfDeg(135.0f);  // 135 degree turn

const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free

// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}

#ifndef FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
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
    obstacle_free_confidence = 0;
    return;
  }

  // compute current color thresholds
  int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  // Use bottom camera dimensions for floor detection
  int32_t floor_count_threshold = oag_floor_count_frac * bottom_camera.output_size.w * bottom_camera.output_size.h;
  float floor_centroid_frac = floor_centroid / (float)bottom_camera.output_size.h / 2.f;
  
  VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);

  // update our safe confidence using color threshold
  if(color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }
  static float prev_time = 0;
  float now = get_sys_time_float();
  float dt = now - prev_time;
  prev_time = now;
  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float speed_sp = fminf(oag_max_speed, 0.2f * obstacle_free_confidence);

  switch (navigation_state){
    case SAFE:
    if (floor_count < floor_count_threshold){
      navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else {
        guidance_h_set_body_vel(speed_sp, 0);
      }

      break;
    case OBSTACLE_FOUND:
      // stop
      guidance_h_set_body_vel(0, 0);

      // randomly select new search direction
      chooseRandomIncrementAvoidance();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 2){
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
      // stop
      guidance_h_set_body_vel(0, 0);

      // Prepare for backing up
      backward_timer = 0;
      navigation_state = BACKING_UP;

      break;
    case BACKING_UP:
      // Move backward for a set time
      guidance_h_set_body_vel(-BACKUP_SPEED, 0);
  
      // Update timer
      backward_timer += dt;
  
      if (backward_timer >= BACKUP_DURATION) {
        // Stop and prepare for rotation
        guidance_h_set_body_vel(0, 0);
    
        // Pick a direction to rotate
        chooseRandomIncrementAvoidance();
    
        // Store current heading
        float current_heading = stateGetNedToBodyEulers_f()->psi;
    
        // Calculate target heading (add or subtract the fixed angle based on direction)
        float target_heading = current_heading + (avoidance_heading_direction * FIXED_ROTATION_ANGLE);
    
        // Normalize to [-pi, pi]
        while (target_heading > M_PI) target_heading -= 2.0f * M_PI;
        while (target_heading < -M_PI) target_heading += 2.0f * M_PI;
    
        // Store target heading for the rotation state to use
        fixed_rotation_angle = target_heading;
    
        navigation_state = ROTATE_FIXED;
      }
      break;
    case ROTATE_FIXED:
      // Get current heading
      float current_heading = stateGetNedToBodyEulers_f()->psi;
      
      // Set desired heading (stored from previous state)
      guidance_h_set_heading(fixed_rotation_angle);
      
      // Calculate angular difference
      float angle_diff = fixed_rotation_angle - current_heading;
      // Normalize to [-pi, pi]
      while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
      
      // Check if we've reached the desired heading (within 5 degrees)
      if (fabsf(angle_diff) < RadOfDeg(5.0f)) {
        navigation_state = REENTER_ARENA;
      }
      break;
    case REENTER_ARENA:
      // Check if we have green floor again or just wait a bit
      if (floor_count > floor_count_threshold || obstacle_free_confidence >= 3) {
      // return to heading mode
      guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
      // reset safe counter
      obstacle_free_confidence = 0;
      // ensure direction is safe before continuing
      navigation_state = SAFE;
      } else {
        // Gradually increase speed to escape boundary
        static float reenter_speed = 0.1f;
        reenter_speed = fminf(reenter_speed + 0.01f, 0.3f);
        guidance_h_set_body_vel(reenter_speed, 0);
        obstacle_free_confidence++;
      }
      break;
    default:
      break;
  }
  return;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    avoidance_heading_direction = 1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  } else {
    avoidance_heading_direction = -1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  }
  return false;
}
