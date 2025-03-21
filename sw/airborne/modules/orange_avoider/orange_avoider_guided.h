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
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the green obstacles but for detecting the floor. When the total amount
 * of floor color drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 *
 * A Region of Interest (ROI) is implemented to only process green objects in the middle part of the bottom camera image.
 * ROI dimensions can be configured in the airframe file using OAG_ROI_HEIGHT and OAG_ROI_WIDTH.
 */

#ifndef ORANGE_AVOIDER_GUIDED_H
#define ORANGE_AVOIDER_GUIDED_H

// settings
extern float oag_color_count_frac;  // obstacle detection threshold as a fraction of total of image
extern float oag_max_speed;         // max flight speed [m/s]
extern float oag_heading_rate;      // heading rate setpoint [rad/s]
extern float oag_heading_change;    // fixed heading change angle for avoidance [rad]

// ROI settings - can be defined in airframe file using OAG_ROI_HEIGHT and OAG_ROI_WIDTH
extern float oag_roi_height;        // height of the ROI as a fraction of image height (from top)
extern float oag_roi_width;         // width of the ROI as a fraction of image width (centered)

// Add handler function for GCS settings
extern void orange_avoider_guided_SetHeadingRate(float val);

extern void orange_avoider_guided_init(void);
extern void orange_avoider_guided_periodic(void);

#endif

