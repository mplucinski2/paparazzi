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
 * The avoidance strategy is to detect horizontal edges where black/blue transitions to green.
 * When the edge_ratio falls below a certain threshold, we assume there is an obstacle and turn.
 *
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * A Region of Interest (ROI) is implemented to only process edges in the specified part of the bottom camera image.
 */

#ifndef ORANGE_AVOIDER_GUIDED_H
#define ORANGE_AVOIDER_GUIDED_H

#include <stdbool.h>

// settings
extern float oag_edge_ratio_threshold;  // edge detection threshold as a ratio (0.0-1.0)
extern float oag_max_speed;             // max flight speed [m/s]
extern float oag_heading_rate;          // heading rate setpoint [rad/s]
extern float oag_color_count_frac;      // kept for compatibility

// ROI settings - can be defined in airframe file
extern float oag_roi_width;             // width of the ROI as a fraction of image width (centered)
extern float oag_roi_height;            // height of the ROI as a fraction of image height (from top)

// Edge detection settings
extern float edge_ratio;                // ratio of columns with detected edges

// Add handler functions for GCS settings
extern void orange_avoider_guided_SetHeadingRate(float val);
extern void orange_avoider_guided_SetColorCountFrac(float val);

extern void orange_avoider_guided_init(void);
extern void orange_avoider_guided_periodic(void);

#endif

