/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * Modified to use optical flow divergence instead of color detection.
 * When an obstacle is detected, the drone turns exactly 90 degrees clockwise.
 */

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H

// settings
extern float oa_color_count_frac;  // kept for backwards compatibility
extern float oa_divergence_threshold; // threshold for optical flow divergence

// functions
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

#endif

