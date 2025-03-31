/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider_guided.c"
 * @author Kirk Scheper, Miłosz Pluciński (modification)
 * added settings for the threshold of green detection threshold, and the external variable (the total number of pixels in the region of interest)
 */

#ifndef ORANGE_AVOIDER_GUIDED_H
#define ORANGE_AVOIDER_GUIDED_H

#include <stdint.h> // For uint32_t type

// settings as before in the orange_avoider_guided.h
extern float oag_color_count_frac;
extern float oag_max_speed;         
extern float oag_heading_rate;     

extern uint32_t current_roi_area;   //actual ROI area calculated from dimensions

//confidence setting
extern int16_t max_trajectory_confidence; 

//function declarations
extern void orange_avoider_guided_SetHeadingRate(float val);
extern uint8_t setInitialAvoidanceDirection(void);

extern void orange_avoider_guided_init(void);
extern void orange_avoider_guided_periodic(void);

#endif

