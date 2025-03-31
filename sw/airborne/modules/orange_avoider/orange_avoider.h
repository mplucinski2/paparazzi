/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens, Miłosz Pluciński (modified)
 */

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H

// settings
extern float oa_color_count_frac;  
extern float oa_divergence_threshold; //threshold for optical flow divergence

// functions
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

#endif

