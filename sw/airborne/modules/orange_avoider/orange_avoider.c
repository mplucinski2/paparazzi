/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * 
 * Modified to use optical flow divergence instead of color detection.
 * The avoidance strategy now uses the divergence calculated from the optical flow.
 * When the divergence is above a certain threshold (given by oa_divergence_threshold),
 * we assume that there is an obstacle in front of the drone and we turn 90 degrees clockwise.
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include "modules/datalink/telemetry.h"

// Include optical flow modules
#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/computer_vision/opticflow/inter_thread_data.h"
#include "modules/computer_vision/opticflow/size_divergence.h"

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE
// Set a less frequent interval for divergence printing to reduce output
#define DIVERGENCE_PRINT_FREQUENCY 20

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Define functions that only print divergence-related info or suppress prints
#define DIVERGENCE_PRINT(string,...) VERBOSE_PRINT(string, ##__VA_ARGS__)
#define SILENT_PRINT(...) do {} while(0)

// Message counter for controlling print frequency
static uint16_t msg_counter = 0;

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static void send_obstacle_confidence(struct transport_tx *trans, struct link_device *dev);
static uint8_t chooseRandomIncrementAvoidance(void);

// Add heading increment variable
float heading_increment = 5.f;          // heading angle increment [deg]

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  TURNING_OBSTACLE, // For 90-degree turns when obstacle is detected
  TURNING_BOUNDARY, // For turning back into arena when boundary is reached
  OUT_OF_BOUNDS
};

// define settings
float oa_color_count_frac = 0.18f;  // kept for backwards compatibility

// Set divergence threshold - use the defined value from airframe.h if available
float oa_divergence_threshold = 0.01f; // default threshold for divergence detection

// define and initialise global variables
enum navigation_state_t navigation_state = TURNING_OBSTACLE;
float divergence = 0.0f;           // divergence value from optical flow
float maxDistance = 2.25;          // max waypoint displacement [m]
float target_heading;              // Target heading after turning 90 degrees
bool turning_complete = false;     // Flag to track if turning is complete

const float HEADING_TOLERANCE = RadOfDeg(3.0f); // Tolerance for heading alignment (3 degrees)

/*
 * This next section defines an ABI messaging event for optical flow.
 * The ABI event is triggered every time new optical flow data is available,
 * and we bind to this event to get the divergence information.
 */
#ifndef ORANGE_AVOIDER_OPTICAL_FLOW_ID
#define ORANGE_AVOIDER_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
static abi_event opticflow_ev;

static void opticflow_cb(uint8_t sender_id __attribute__((unused)), 
                        uint32_t stamp __attribute__((unused)),
                        int flow_x __attribute__((unused)), 
                        int flow_y __attribute__((unused)),
                        int flow_der_x __attribute__((unused)), 
                        int flow_der_y __attribute__((unused)),
                        float quality __attribute__((unused)), 
                        float size_divergence)
{
  // Only process divergence in SAFE state - ignore during all turning states
  if (navigation_state == SAFE) {
    divergence = size_divergence;
    
    // Print divergence values at the specified frequency
    static uint8_t flow_msg_counter = 0;
    flow_msg_counter++;
    
    if (flow_msg_counter % DIVERGENCE_PRINT_FREQUENCY == 0) {
      // Only print every DIVERGENCE_PRINT_FREQUENCY calls
      DIVERGENCE_PRINT("Divergence value from optical flow: %f\n", divergence);
    }
  }
}

/*
 * Initialisation function, setting up ABI bindings for optical flow, random seed and heading_increment
 */
void orange_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our opticflow callback to receive the opticflow results
  AbiBindMsgOPTICAL_FLOW(ORANGE_AVOIDER_OPTICAL_FLOW_ID, &opticflow_ev, opticflow_cb);
  
  // Register telemetry for obstacle_free_confidence
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OBSTACLE_CONFIDENCE, send_obstacle_confidence);
  
  DIVERGENCE_PRINT("Orange Avoider initialized with divergence threshold: %f\n", oa_divergence_threshold);
}

/**
 * Send telemetry data for obstacle detection
 * This function is called by the telemetry module, but we'll only send data periodically
 */
static void send_obstacle_confidence(struct transport_tx *trans, struct link_device *dev)
{
  // Send telemetry more frequently to match our divergence print frequency
  static uint8_t telemetry_counter = 0;
  telemetry_counter++;
  
  // Send more frequently (every DIVERGENCE_PRINT_FREQUENCY) or during important events
  if (telemetry_counter % DIVERGENCE_PRINT_FREQUENCY == 0 || 
      navigation_state == TURNING_OBSTACLE || 
      navigation_state == OBSTACLE_FOUND || 
      navigation_state == TURNING_BOUNDARY || 
      navigation_state == OUT_OF_BOUNDS) {
    // Create a dummy confidence value of 1/0 based on divergence vs threshold
    // to maintain compatibility with existing telemetry message
    int16_t obstacle_detected = (divergence > oa_divergence_threshold) ? 0 : 1;
    uint8_t nav_state = (uint8_t)navigation_state;
    pprz_msg_send_OBSTACLE_CONFIDENCE(trans, dev, AC_ID, 
                                    &obstacle_detected,
                                    &divergence,
                                    &oa_divergence_threshold,
                                    &nav_state);
  }
}

/*
 * Check if the current heading is close to the target heading
 */
static bool is_heading_aligned(void)
{
  // Use nav.heading instead of the actual measured heading (which might lag)
  float current_heading = nav.heading;
  float diff = fabsf(current_heading - target_heading);
  
  // Normalize the difference to [0, π]
  if (diff > M_PI) {
    diff = 2 * M_PI - diff;
  }
  
  return diff < HEADING_TOLERANCE;
}

/*
 * Function that implements the obstacle avoidance logic
 */
void orange_avoider_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  // Increment counter
  msg_counter++;
  
  // Print divergence information at the specified frequency
  if (msg_counter % DIVERGENCE_PRINT_FREQUENCY == 0) {
    DIVERGENCE_PRINT("Divergence: %f  threshold: %f state: %d\n", 
               divergence, oa_divergence_threshold, navigation_state);
  }

  // Calculate move distance - fixed distance
  float moveDistance = 1.0f;  // Use a reasonable constant distance

  switch (navigation_state){
    case SAFE:
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
      
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else if (divergence > oa_divergence_threshold){
        // Direct threshold crossing detection - exactly as described in the guide
        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
        moveWaypointForward(WP_RETREAT, -1.0f * moveDistance);
      }
      break;
      
    case OBSTACLE_FOUND:
      // Stop the drone
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_RETREAT);
      waypoint_move_here_2d(WP_TRAJECTORY);

      // Set target heading 90 degrees clockwise from current heading
      float current_heading = stateGetNedToBodyEulers_f()->psi;
      target_heading = current_heading - RadOfDeg(90.0f);  // Clockwise is negative in Paparazzi
      
      // Normalize heading to [-pi, pi]
      FLOAT_ANGLE_NORMALIZE(target_heading);
      
      // Set the new heading
      nav.heading = target_heading;
      
      // Only print divergence and critical state changes
      DIVERGENCE_PRINT("CRITICAL: Obstacle detected! Divergence: %f Threshold: %f\n", 
                      divergence, oa_divergence_threshold);
      
      // Reset turning flag
      turning_complete = false;
      
      // Move to TURNING_OBSTACLE state
      navigation_state = TURNING_OBSTACLE;
      break;
      
    case TURNING_OBSTACLE:
      // Check if we've reached the target heading
      if (is_heading_aligned()) {
        if (!turning_complete) {
          turning_complete = true;
          // Print only critical state changes
          DIVERGENCE_PRINT("CRITICAL: Turn complete after obstacle, new heading: %f\n", 
                          DegOfRad(stateGetNedToBodyEulers_f()->psi));
          
          // Explicitly move waypoints in the new direction to get the drone moving
          float initialMoveDistance = 1.5f;  // Use a reasonable initial distance
          moveWaypointForward(WP_TRAJECTORY, 2.0f * initialMoveDistance);
          moveWaypointForward(WP_GOAL, initialMoveDistance);
          moveWaypointForward(WP_RETREAT, -0.5f * initialMoveDistance);
        }
        
        // Return to SAFE state once the turn is complete
        navigation_state = SAFE;
      }
      break;
      
    case OUT_OF_BOUNDS:
      // When out of bounds, apply heading increment to turn back
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);
      moveWaypointForward(WP_RETREAT, -1.0f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // Add extra heading increment to point back into arena
        increase_nav_heading(heading_increment);
        
        SILENT_PRINT("Back inside boundary, heading: %f\n", DegOfRad(nav.heading));
        
        // Transition to TURNING_BOUNDARY
        navigation_state = TURNING_BOUNDARY;
      }
      break;
      
    case TURNING_BOUNDARY:
      // Move forward after turning at boundary
      float boundaryMoveDistance = 1.0f;
      moveWaypointForward(WP_TRAJECTORY, 2.0f * boundaryMoveDistance);
      moveWaypointForward(WP_GOAL, boundaryMoveDistance);
      moveWaypointForward(WP_RETREAT, -0.5f * boundaryMoveDistance);
      
      SILENT_PRINT("Moving waypoints after boundary turn, heading: %f\n", DegOfRad(nav.heading));
      
      // Return to SAFE state
      navigation_state = SAFE;
      break;
      
    default:
      break;
  }
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  // Suppress prints
  SILENT_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  // Use the navigation heading rather than the body heading
  float heading = nav.heading;
  
  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  
  // Suppress prints
  SILENT_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
              POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
              stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  // Suppress prints
  SILENT_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = 5.f;
    SILENT_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -5.f;
    SILENT_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}

