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

