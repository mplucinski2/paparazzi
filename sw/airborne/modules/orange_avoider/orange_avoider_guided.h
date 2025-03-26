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
 * ROI dimensions are configured directly in the airframe file using COLOR_OBJECT_DETECTOR_ROI_* settings.
 */

#ifndef ORANGE_AVOIDER_GUIDED_H
#define ORANGE_AVOIDER_GUIDED_H

#include <stdint.h> // For uint32_t type

// settings
extern float oag_color_count_frac;  // obstacle detection threshold as a fraction of total of image
extern float oag_max_speed;         // max flight speed [m/s]
extern float oag_heading_rate;      // heading rate setpoint [rad/s]

// Fixed scan area setting
extern uint32_t oag_fixed_scan_area; // Fixed scan area in pixels for obstacle detection
extern uint32_t current_roi_area;    // Actual ROI area calculated from dimensions

// Function declarations
extern void orange_avoider_guided_SetHeadingRate(float val);
extern void orange_avoider_guided_SetFixedScanArea(float val);
extern uint8_t setInitialAvoidanceDirection(void);

extern void orange_avoider_guided_init(void);
extern void orange_avoider_guided_periodic(void);

#endif

