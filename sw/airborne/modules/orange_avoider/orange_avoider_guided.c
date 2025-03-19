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

// #include "modules/orange_avoider/orange_avoider_guided.h"
// #include "firmwares/rotorcraft/guidance/guidance_h.h"
// #include "generated/airframe.h"
// #include "state.h"
// #include "modules/core/abi.h"
// #include <stdio.h>
// #include <time.h>
// #include "math/pprz_geodetic.h"  


// #define ORANGE_AVOIDER_VERBOSE TRUE

// #define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
// #if ORANGE_AVOIDER_VERBOSE
// #define VERBOSE_PRINT PRINT
// #else
// #define VERBOSE_PRINT(...)
// #endif
// #define FIXED_ROTATION_ANGLE RadOfDeg(10.f)  // Rotate 30 degrees

// uint8_t chooseRandomIncrementAvoidance(void);

// enum navigation_state_t {
//   SAFE,
//   OBSTACLE_FOUND,
//   SEARCH_FOR_SAFE_HEADING,
//   OUT_OF_BOUNDS,
//   REENTER_ARENA
// };

// // define settings
// float oag_color_count_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
// float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
// float oag_max_speed = 0.5f;               // max flight speed [m/s]
// float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]

// // define and initialise global variables
// enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
// int32_t color_count = 0;                // orange color count from color filter for obstacle detection
// int32_t floor_count = 0;                // green color count from color filter for floor detection
// int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
// float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
// int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.
// float target_heading = 0;  // Define global target heading

// const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free

// // This call back will be used to receive the color count from the orange detector
// #ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
// #error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
// #error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
// #endif
// static abi_event color_detection_ev;
// static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
//                                int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
//                                int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
//                                int32_t quality, int16_t __attribute__((unused)) extra)
// {
//   color_count = quality;
// }

// #ifndef FLOOR_VISUAL_DETECTION_ID
// #error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
// #error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
// #endif
// static abi_event floor_detection_ev;
// static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
//                                int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
//                                int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
//                                int32_t quality, int16_t __attribute__((unused)) extra)
// {
//   floor_count = quality;
//   floor_centroid = pixel_y;
// }



// /*
//  * Initialisation function
//  */
// void orange_avoider_guided_init(void)
// {
//   // Initialise random values
//   srand(time(NULL));
//   chooseRandomIncrementAvoidance();

//   // bind our colorfilter callbacks to receive the color filter outputs
//   AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
//   AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
// }

// /*
//  * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
//  */
// void orange_avoider_guided_periodic(void)
// {
//   // Only run the mudule if we are in the correct flight mode
//   if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
//     navigation_state = SEARCH_FOR_SAFE_HEADING;
//     obstacle_free_confidence = 0;
//     return;
//   }

//   // compute current color thresholds
//   int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
//   int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
//   float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

//   VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
//   VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
//   VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);

//   // update our safe confidence using color threshold
//   if(color_count < color_count_threshold){
//     obstacle_free_confidence++;
//   } else {
//     obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
//   }

//   // bound obstacle_free_confidence
//   Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

//   float speed_sp = fminf(oag_max_speed, 0.2f * obstacle_free_confidence);
//   float current_heading = stateGetNedToBodyEulers_f()->psi;
  
//   switch (navigation_state){
//     case SAFE:
//       if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
//         navigation_state = OUT_OF_BOUNDS;
//       } else if (obstacle_free_confidence == 0){
//         navigation_state = OBSTACLE_FOUND;
//       } else {
//         guidance_h_set_body_vel(speed_sp, 0);
//       }

//       break;
//     case OBSTACLE_FOUND:
//       // stop
//       guidance_h_set_body_vel(0, 0);
    
//       // Randomly select new search direction
//       float rotation_sign = (rand() % 2 == 0) ? 1.f : -1.f;  // Random CW or CCW
//       target_heading = current_heading + rotation_sign * FIXED_ROTATION_ANGLE;
    
//       navigation_state = SEARCH_FOR_SAFE_HEADING;
//       break;
//     case SEARCH_FOR_SAFE_HEADING:
//       float angle_difference = fmodf(target_heading - current_heading + M_PI, 2 * M_PI) - M_PI;

//     static int stuck_counter = 0;
//     if (fabsf(angle_difference) < RadOfDeg(5.0) ) {  //& & obstacle_free_confidence > 2
//       guidance_h_set_heading(target_heading);
//       guidance_h_set_body_vel(oag_max_speed, 0);
//       navigation_state = SAFE;
//       stuck_counter = 0;
//     } else {
//       guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);
//       stuck_counter++;
//     }
//     // If stuck rotating too long, switch avoidance direction 
//     if (stuck_counter > 10) {  
//       avoidance_heading_direction *= -1;  // Flip direction
//       target_heading = current_heading + avoidance_heading_direction * FIXED_ROTATION_ANGLE;
//       stuck_counter = 0;  // Reset counter
//     }
  
  
//       break;
//     case OUT_OF_BOUNDS:
//       // stop
//       guidance_h_set_body_vel(0, 0);

//       // start turn back into arena
//       guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(15));

//       navigation_state = REENTER_ARENA;

//       break;
//     case REENTER_ARENA:
//       // force floor center to opposite side of turn to head back into arena
//       if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
//         // return to heading mode
//         guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);

//         // reset safe counter
//         obstacle_free_confidence = 0;

//         // ensure direction is safe before continuing
//         navigation_state = SAFE;
//       }
//       break;
//     default:
//       break;
//   }
//   return;
// }

// /*
//  * Sets the variable 'incrementForAvoidance' randomly positive/negative
//  */
// uint8_t chooseRandomIncrementAvoidance(void)
// {
//   // Randomly choose CW or CCW avoiding direction
//   if (rand() % 2 == 0) {
//     avoidance_heading_direction = 1.f;
//     VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
//   } else {
//     avoidance_heading_direction = -1.f;
//     VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
//   }
//   return false;
// }

// ################################ New Script ################################
// This module will implement a new state machine capable of handling the following states, based on inputs from
// the optical flow calculations and the color filter of the bottom camera. The objective is to avoid all obstacles
// to stay within the bounds of the floor and to also detect the base of obstacles with the bottom camera.
// The state machine will be as follows:
//
// SAFE: The drone is flying safely and there are no obstacles in the way.
// OBSTACLE_FOUND_OPTICAL_FLOW: The threshold for optical flow has been exceeded and we assume that there is an obstacle in the way.
// Here the drone should slow down and turn away until the point of divergence is out of the frame.
// OBSTACLE_FOUND_BOTTOM_CAM:  The floor detection has been triggered by the base of an obstacle which is visible in the edges of the bottom camera view.
// Here the drone should stop and turn away from the obstacle, until it is not detected within the frame (ground is green again).
// OUT_OF_BOUNDS: The floor detection has been triggered and the drone is near the edge of the zoo.
// REENTER_ARENA: The drone will rotate until the floor detection is no longer triggered and then return to the SAFE state.


#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>

defineuint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND_OPTICAL_FLOW,
  OBSTACLE_FOUND_BOTTOM,
  OUT_OF_BOUNDS,
  REENTER_ARENA  
};

// define settings
//Obstacle detection threshold as a fraction of total of image
float oag_bottom_cam_count_frac = 0.03f;       // obstacle detection threshold as a fraction of total of image
//Floor detection threshold as a fraction of total of image
float oag_color_count_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
//optical flow threshold
float oag_optical_flow_threshold = 0.1f;  // optical flow threshold for obstacle detection
//Velocity
float oag_optical_flow_speed = 0.2f;      // speed for optical flow obstacle avoidance
float oag_max_speed = 0.5f;               // max flight speed [m/s]
//Heading
float oag_heading_rate = RadOfDeg(5.f);  // heading change setpoint for avoidance [rad/s]

// define and initialise global variables

enum navigation_state_t navigation_state = SAFE;   // current state in state machine
int32_t optical_flow_count = 0;                // optical flow count from optical flow detector for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
int32_t bottom_cam_count = 0;          // orange color count from color filter for obstacle detection
int32_t bottom_cam_obst_loc = 0;       // location of obstacle in bottom camera view ; 0 for first quadrant, 1 for second, 2 for third and 3 for fourth
// Array to store coordinates
int32_t coords_divergence[2] = {0, 0};

//Call back for floor visual detection
#ifndef FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the floor filter 
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

//Placeholder for bottom camera detection. The frame is split into 4 quadrants and the number of non-green pixels in each quadrant is counted.
#ifndef BOTTOM_CAM_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define BOTTOM_CAM_VISUAL_DETECTION_ID to the bottom camera filter
#endif
static abi_event bottom_cam_detection_ev;
static void bottom_cam_detection_cb(int32_t upper_left, int32_t upper_right, int32_t lower_left, int32_t lower_right)
{
  bottom_cam_count = upper_left + upper_right + lower_left + lower_right;
  //check which quadrant has the most non-green pixels
  if (upper_left > upper_right && upper_left > lower_left && upper_left > lower_right){
    bottom_cam_obst_loc = 0;
  } else if (upper_right > upper_left && upper_right > lower_left && upper_right > lower_right){
    bottom_cam_obst_loc = 1;
  } else if (lower_left > upper_left && lower_left > upper_right && lower_left > lower_right){
    bottom_cam_obst_loc = 2;
  } else {
    bottom_cam_obst_loc = 3;
  }
  
}

//Placeholder for optical flow detection. The number of pixels with a high optical flow is counted.
static abi_event optical_flow_detection_ev;
static void optical_flow_detection_cb(float quality, int32_t divergence_x, int32_t divergence_y)
{
  optical_flow_count = quality;

}
{
  optical_flow_count = quality;
  coords_divergence[0] = divergence_x;
  coords_divergence[1] = divergence_y;
}


/*
 * Initialisation function
 */
void new_script_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
  AbiBindMsgVISUAL_DETECTION(BOTTOM_CAM_VISUAL_DETECTION_ID, &bottom_cam_detection_ev, bottom_cam_detection_cb);
  AbiBindMsgVISUAL_DETECTION(OPTICAL_FLOW_VISUAL_DETECTION_ID, &optical_flow_detection_ev, optical_flow_detection_cb);
}

/*
 * Periodic function responsible for updating the variables and state machine
 */

void new_script_periodic(void)
{ 
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SAFE;
    return;
  }

  // compute current color thresholds
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t bottom_cam_threshold = oag_bottom_cam_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  float optical_flow_threshold = oag_optical_flow_threshold // Do we need to multiply by the size of the image?
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  float current_heading = stateGetNedToBodyEulers_f()->psi;

  switch (navigation_state){
    case SAFE:
      if (optical_flow_count > optical_flow_threshold) {
      navigation_state = OBSTACLE_FOUND_OPTICAL_FLOW;
      } else if (bottom_cam_count > bottom_cam_threshold) {
      navigation_state = OBSTACLE_FOUND_BOTTOM;
      } else if (floor_count > floor_count_threshold) {
      navigation_state = OUT_OF_BOUNDS;
      }
      break;

    case OBSTACLE_FOUND_OPTICAL_FLOW:
      if (optical_flow_count <= optical_flow_threshold) {
      navigation_state = SAFE;
      } else {
      // Implement avoidance maneuver based on divergence coordinates
      guidance_h_set_guided_body_vel(0, -oag_optical_flow_speed, 0);
      guidance_h_set_guided_heading(current_heading + oag_heading_rate);
      }
      break;

    case OBSTACLE_FOUND_BOTTOM:
      if (bottom_cam_count <= bottom_cam_threshold) {
      navigation_state = SAFE;
      } else {
      // Implement avoidance maneuver based on obstacle location in bottom camera
      switch (bottom_cam_obst_loc) {
        case 0:
        guidance_h_set_guided_body_vel(-oag_optical_flow_speed, 0, 0);
        break;
        case 1:
        guidance_h_set_guided_body_vel(oag_optical_flow_speed, 0, 0);
        break;
        case 2:
        guidance_h_set_guided_body_vel(0, -oag_optical_flow_speed, 0);
        break;
        case 3:
        guidance_h_set_guided_body_vel(0, oag_optical_flow_speed, 0);
        break;
      }
      guidance_h_set_guided_heading(current_heading + oag_heading_rate);
      }
      break;

    case OUT_OF_BOUNDS:
      if (floor_count <= floor_count_threshold) {
      navigation_state = SAFE;
      } else {
      // Implement reenter arena maneuver
      guidance_h_set_guided_body_vel(0, -oag_optical_flow_speed, 0);
      guidance_h_set_guided_heading(current_heading + oag_heading_rate);
      }
      break;

    case REENTER_ARENA:
      if (floor_count <= floor_count_threshold) {
      navigation_state = SAFE;
      } else {
      // Implement reenter arena maneuver
      guidance_h_set_guided_body_vel(0, -oag_optical_flow_speed, 0);
      guidance_h_set_guided_heading(current_heading + oag_heading_rate);
      }
      break;

    default:
      navigation_state = SAFE;
      break;


  }



}

