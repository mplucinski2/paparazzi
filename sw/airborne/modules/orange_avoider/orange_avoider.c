/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * using optic flow divergence to detect obstacles, as from the theory time-to-contact is inversely proportional to the divergence, the derotation parameters and others are specified
 * as in the crash course. The middle ROI is used for divergence calculation. If the divergence crosses certain threshold, the obstacle is detected. When obstacle is detected, the mav changes heading by 90 degrees
 * and continues to fly straight (checking flow when stationary, and looking for safe heading does not work, as divergence requires translational motion). Overall this code does not work well
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

#define NAV_C
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE
#define DIVERGENCE_PRINT_FREQUENCY 20

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#define DIVERGENCE_PRINT(string,...) VERBOSE_PRINT(string, ##__VA_ARGS__)
#define SILENT_PRINT(...) do {} while(0)

//controlling printin frequency
static uint16_t msg_counter = 0;

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static void send_obstacle_confidence(struct transport_tx *trans, struct link_device *dev);
static uint8_t chooseRandomIncrementAvoidance(void);

float heading_increment = 5.f;          //heading rate

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  TURNING_OBSTACLE, //90 degree turn when obstacle is detected
  TURNING_BOUNDARY, //turning when out of bounds
  OUT_OF_BOUNDS
};

float oa_color_count_frac = 0.18f;  //kept for backwards compatibility with the original module
float oa_divergence_threshold = 0.01f; // default obstacle detection threshold for optic flow divergence

//defining and initialising global variables
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
  //do not compute divergence when turning, as mentioned, it does not make sense
  if (navigation_state == SAFE) {
    divergence = size_divergence;
    
    //printing divergence at a given frequency
    static uint8_t flow_msg_counter = 0;
    flow_msg_counter++;
    
    if (flow_msg_counter % DIVERGENCE_PRINT_FREQUENCY == 0) {
      // Only print every DIVERGENCE_PRINT_FREQUENCY calls
      DIVERGENCE_PRINT("Divergence value from optical flow: %f\n", divergence);
    }
  }
}

/*
 *init function and setting abi binding
 */
void orange_avoider_init(void)
{
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  //receiving optical flow data from the opticflow module
  AbiBindMsgOPTICAL_FLOW(ORANGE_AVOIDER_OPTICAL_FLOW_ID, &opticflow_ev, opticflow_cb);
  
  //registering telemetry for obstacle confidence
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OBSTACLE_CONFIDENCE, send_obstacle_confidence);
  
  DIVERGENCE_PRINT("initialized with divergence threshold: %f\n", oa_divergence_threshold);
}

/**
 *sending telemetry data for obstacle detection
 */
static void send_obstacle_confidence(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t telemetry_counter = 0;
  telemetry_counter++;
  
  //send telemetry more frequently or during important events
  if (telemetry_counter % DIVERGENCE_PRINT_FREQUENCY == 0 || 
      navigation_state == TURNING_OBSTACLE || 
      navigation_state == OBSTACLE_FOUND || 
      navigation_state == TURNING_BOUNDARY || 
      navigation_state == OUT_OF_BOUNDS) {
    //creating dummy confidence to just turn when detecting an obstacle, no repeated detections needed for the speed of response
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
 *checking if current heading is close to target heading
 */
static bool is_heading_aligned(void)
{
  //using nav.heading instead of the actual measured heading
  float current_heading = nav.heading;
  float diff = fabsf(current_heading - target_heading);
  
  //normalizing the difference to [0, π]
  if (diff > M_PI) {
    diff = 2 * M_PI - diff;
  }
  
  return diff < HEADING_TOLERANCE;
}

/*
 *function implementing the obstacle avoidance logic
 */
void orange_avoider_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  //counter
  msg_counter++;
  
  //printing divergence at a given frequency
  if (msg_counter % DIVERGENCE_PRINT_FREQUENCY == 0) {
    DIVERGENCE_PRINT("Divergence: %f  threshold: %f state: %d\n", 
               divergence, oa_divergence_threshold, navigation_state);
  }

  //setting a constant determining how far to move (for next waypoint)
  float moveDistance = 1.0f;  

  switch (navigation_state){
    case SAFE:
      //moving waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
      
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else if (divergence > oa_divergence_threshold){
        //setting the state to OBSTACLE_FOUND when divergence crosses threshold
        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
        moveWaypointForward(WP_RETREAT, -1.0f * moveDistance);
      }
      break;
      
    case OBSTACLE_FOUND:
      //stopping the drone
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_RETREAT);
      waypoint_move_here_2d(WP_TRAJECTORY);

      //setting target heading 90 degrees clockwise from current heading
      float current_heading = stateGetNedToBodyEulers_f()->psi;
      target_heading = current_heading - RadOfDeg(90.0f);  
      
      //normalizing the heading to [-pi, pi]
      FLOAT_ANGLE_NORMALIZE(target_heading);
      
      //setting the new heading
      nav.heading = target_heading;
      
      //only printing divergence and critical state changes
      DIVERGENCE_PRINT("CRITICAL: Obstacle detected! Divergence: %f Threshold: %f\n", 
                      divergence, oa_divergence_threshold);
      
      turning_complete = false;
      //moving to the state of turning when obstacle is detected
      navigation_state = TURNING_OBSTACLE;
      break;
      
    case TURNING_OBSTACLE:
      //checking if target heading is reached
      if (is_heading_aligned()) {
        if (!turning_complete) {
          turning_complete = true;
          DIVERGENCE_PRINT("CRITICAL: Turn complete after obstacle, new heading: %f\n", 
                          DegOfRad(stateGetNedToBodyEulers_f()->psi));
          
          //setting the waypoints in the new direction
          float initialMoveDistance = 1.5f; 
          moveWaypointForward(WP_TRAJECTORY, 2.0f * initialMoveDistance);
          moveWaypointForward(WP_GOAL, initialMoveDistance);
          moveWaypointForward(WP_RETREAT, -0.5f * initialMoveDistance);
        }
        
        //when the turn is complete, the state is set to safe (the mav will continue to fly straight)
        navigation_state = SAFE;
      }
      break;
      
    case OUT_OF_BOUNDS:
      //heading increment to turn back when out of bounds
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);
      moveWaypointForward(WP_RETREAT, -1.0f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        //extra heading increment to point back into arena
        increase_nav_heading(heading_increment);
        
        SILENT_PRINT("Back inside boundary, heading: %f\n", DegOfRad(nav.heading));
        
        //transitioning to the state of turning when out of bounds
        navigation_state = TURNING_BOUNDARY;
      }
      break;
      
    case TURNING_BOUNDARY:
      //moving forward after completing the turn
      float boundaryMoveDistance = 1.0f;
      moveWaypointForward(WP_TRAJECTORY, 2.0f * boundaryMoveDistance);
      moveWaypointForward(WP_GOAL, boundaryMoveDistance);
      moveWaypointForward(WP_RETREAT, -0.5f * boundaryMoveDistance);
      
      SILENT_PRINT("Moving waypoints after boundary turn, heading: %f\n", DegOfRad(nav.heading));
      
      //again, returning to the safe state
      navigation_state = SAFE;
      break;
      
    default:
      break;
  }
}

/*
 * function increasing the nav heading
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  //normalizing the heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  //set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  SILENT_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 *calculating the coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * calculating the coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  // Use the navigation heading rather than the body heading
  float heading = nav.heading;
  
  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  
  SILENT_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
              POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
              stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 *moving waypoint to a new coordinate
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  SILENT_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 *setting heading increment randomly
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

