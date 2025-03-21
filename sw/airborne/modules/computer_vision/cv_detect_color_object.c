/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;

// Edge detection settings - default values match the black/blue and green colors from earlier code
uint8_t cod_top_color_lum_min1 = 23;    // Black/blue color (top part of edge)
uint8_t cod_top_color_lum_max1 = 90;
uint8_t cod_top_color_cb_min1 = 110;
uint8_t cod_top_color_cb_max1 = 185;
uint8_t cod_top_color_cr_min1 = 60;
uint8_t cod_top_color_cr_max1 = 140;

uint8_t cod_bottom_color_lum_min1 = 60;  // Green color (bottom part of edge)
uint8_t cod_bottom_color_lum_max1 = 130;
uint8_t cod_bottom_color_cb_min1 = 70;
uint8_t cod_bottom_color_cb_max1 = 102;
uint8_t cod_bottom_color_cr_min1 = 110;
uint8_t cod_bottom_color_cr_max1 = 130;

bool cod_edge_detection1 = false;
bool cod_edge_detection2 = false;

// ROI parameters for camera 1 (used for green detection)
uint16_t roi_x_min1 = 80;  // Default ROI parameters (80x40 = 3200 pixels)
uint16_t roi_x_max1 = 160;
uint16_t roi_y_min1 = 0;
uint16_t roi_y_max1 = 40;
bool use_roi1 = true;      // Use ROI for camera 1 by default

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
};
struct color_object_t global_filters[2];

// Functions
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);

uint32_t find_edge_transitions(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                                uint8_t top_lum_min, uint8_t top_lum_max,
                                uint8_t top_cb_min, uint8_t top_cb_max,
                                uint8_t top_cr_min, uint8_t top_cr_max,
                                uint8_t bottom_lum_min, uint8_t bottom_lum_max,
                                uint8_t bottom_cb_min, uint8_t bottom_cb_max,
                                uint8_t bottom_cr_min, uint8_t bottom_cr_max);

/*
 * Check if a pixel's YUV values are within the defined color range
 */
bool is_color_in_range(uint8_t y, uint8_t u, uint8_t v, 
                       uint8_t y_min, uint8_t y_max, 
                       uint8_t u_min, uint8_t u_max, 
                       uint8_t v_min, uint8_t v_max)
{
  return (y >= y_min && y <= y_max &&
          u >= u_min && u <= u_max &&
          v >= v_min && v <= v_max);
}

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;
  bool edge_mode = false;
  
  // Set up parameters based on filter
  switch (filter){
    case 1:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      draw = cod_draw1;
      edge_mode = cod_edge_detection1;
      break;
    case 2:
      lum_min = cod_lum_min2;
      lum_max = cod_lum_max2;
      cb_min = cod_cb_min2;
      cb_max = cod_cb_max2;
      cr_min = cod_cr_min2;
      cr_max = cod_cr_max2;
      draw = cod_draw2;
      edge_mode = cod_edge_detection2;
      break;
    default:
      return img;
  };

  int32_t x_c, y_c;
  uint32_t count;

  // Use either standard color detection or edge detection based on settings
  if (!edge_mode) {
    // Standard color detection
    count = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
    
    // Log which camera is detecting colors and how many pixels were found
    if (filter == 1) {
      VERBOSE_PRINT("Bottom camera (1): found %d color pixels in %s\n", 
                   count, use_roi1 ? "ROI" : "full frame");
    } else {
      VERBOSE_PRINT("Front camera (2): found %d color pixels\n", count);
    }
  } else {
    // Edge detection mode (transitions from one color to another)
    if (filter == 1) {
      // For filter 1, use edge detection settings
      count = find_edge_transitions(img, &x_c, &y_c, draw,
                                   cod_top_color_lum_min1, cod_top_color_lum_max1,
                                   cod_top_color_cb_min1, cod_top_color_cb_max1,
                                   cod_top_color_cr_min1, cod_top_color_cr_max1,
                                   cod_bottom_color_lum_min1, cod_bottom_color_lum_max1,
                                   cod_bottom_color_cb_min1, cod_bottom_color_cb_max1,
                                   cod_bottom_color_cr_min1, cod_bottom_color_cr_max1);
      
      VERBOSE_PRINT("Bottom camera (1): found %d edge transitions in %s\n", 
                   count, use_roi1 ? "ROI" : "full frame");
    } else {
      // For other filters, just use standard color detection as fallback
      count = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
    }
  }

  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = count;
  global_filters[filter-1].x_c = x_c;
  global_filters[filter-1].y_c = y_c;
  global_filters[filter-1].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

struct image_t *object_detector2(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 2);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
  cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
  cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
  cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
  cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
  cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
  cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif

  // Initialize ROI parameters from defines if available
#ifdef COLOR_OBJECT_DETECTOR_ROI_X_MIN1
  roi_x_min1 = COLOR_OBJECT_DETECTOR_ROI_X_MIN1;
  roi_x_max1 = COLOR_OBJECT_DETECTOR_ROI_X_MAX1;
  roi_y_min1 = COLOR_OBJECT_DETECTOR_ROI_Y_MIN1;
  roi_y_max1 = COLOR_OBJECT_DETECTOR_ROI_Y_MAX1;
  use_roi1 = true;
#endif

  // Initialize edge detection settings if available
#ifdef COLOR_OBJECT_DETECTOR_EDGE_DETECTION1
  cod_edge_detection1 = COLOR_OBJECT_DETECTOR_EDGE_DETECTION1;
#endif

#ifdef COLOR_OBJECT_DETECTOR_TOP_COLOR_LUM_MIN1
  cod_top_color_lum_min1 = COLOR_OBJECT_DETECTOR_TOP_COLOR_LUM_MIN1;
  cod_top_color_lum_max1 = COLOR_OBJECT_DETECTOR_TOP_COLOR_LUM_MAX1;
  cod_top_color_cb_min1 = COLOR_OBJECT_DETECTOR_TOP_COLOR_CB_MIN1;
  cod_top_color_cb_max1 = COLOR_OBJECT_DETECTOR_TOP_COLOR_CB_MAX1;
  cod_top_color_cr_min1 = COLOR_OBJECT_DETECTOR_TOP_COLOR_CR_MIN1;
  cod_top_color_cr_max1 = COLOR_OBJECT_DETECTOR_TOP_COLOR_CR_MAX1;
#endif

#ifdef COLOR_OBJECT_DETECTOR_BOTTOM_COLOR_LUM_MIN1
  cod_bottom_color_lum_min1 = COLOR_OBJECT_DETECTOR_BOTTOM_COLOR_LUM_MIN1;
  cod_bottom_color_lum_max1 = COLOR_OBJECT_DETECTOR_BOTTOM_COLOR_LUM_MAX1;
  cod_bottom_color_cb_min1 = COLOR_OBJECT_DETECTOR_BOTTOM_COLOR_CB_MIN1;
  cod_bottom_color_cb_max1 = COLOR_OBJECT_DETECTOR_BOTTOM_COLOR_CB_MAX1;
  cod_bottom_color_cr_min1 = COLOR_OBJECT_DETECTOR_BOTTOM_COLOR_CR_MIN1;
  cod_bottom_color_cr_max1 = COLOR_OBJECT_DETECTOR_BOTTOM_COLOR_CR_MAX1;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
  cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
  cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
  cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
  cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
  cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
  cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
  cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_EDGE_DETECTION2
  cod_edge_detection2 = COLOR_OBJECT_DETECTOR_EDGE_DETECTION2;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2, 1);
#endif
}

/*
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 * Now identifies camera based on YUV values and applies ROI for camera 1.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param draw - whether or not to draw on image
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @return number of pixels of image within the filter bounds.
 */
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

  // Check if this is camera 1 (bottom camera) by comparing against its color settings
  bool is_camera1 = (lum_min == cod_lum_min1 && lum_max == cod_lum_max1 &&
                    cb_min == cod_cb_min1 && cb_max == cod_cb_max1 &&
                    cr_min == cod_cr_min1 && cr_max == cod_cr_max1);
  
  // Set scan region - either full frame or ROI for camera 1
  uint16_t x_start = 0;
  uint16_t x_end = img->w;
  uint16_t y_start = 0;
  uint16_t y_end = img->h;
  
  // For camera 1 (bottom camera) with ROI enabled, only scan the specified ROI
  if (is_camera1 && use_roi1) {
    x_start = roi_x_min1;
    x_end = roi_x_max1;
    y_start = roi_y_min1;
    y_end = roi_y_max1;
    VERBOSE_PRINT("Using ROI for bottom camera: x=%d-%d, y=%d-%d (%d pixels total)\n", 
                  x_start, x_end, y_start, y_end, (x_end-x_start)*(y_end-y_start));
  }

  // Go through the pixels in the specified region
  for (uint16_t y = y_start; y < y_end && y < img->h; y++) {
    for (uint16_t x = x_start; x < x_end && x < img->w; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
        cnt ++;
        tot_x += x;
        tot_y += y;
        if (draw){
          *yp = 255;  // make pixel brighter in image
        }
      }
    }
  }
  if (cnt > 0) {
    *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
    *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }
  
  // For debugging
  if (is_camera1) {
    // Only print debug message every 10 frames (reduces frequency)
    static uint8_t debug_counter = 0;
    if (debug_counter++ % 50 == 0) {
      VERBOSE_PRINT("Bottom camera detected %d green pixels out of %d in ROI (%d%%)\n", 
                   cnt, (x_end-x_start)*(y_end-y_start), (int)(100.0*cnt/((x_end-x_start)*(y_end-y_start))));
    }
  }
  
  return cnt;
}

void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        0, 0, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
        0, 0, local_filters[1].color_count, 1);
    local_filters[1].updated = false;
  }
}

/**
 * Find edge transitions in an image (vertical black/blue -> green transitions)
 * 
 * @param img - input image to process formatted as YUV422
 * @param p_xc - x coordinate of the centroid of detected edges
 * @param p_yc - y coordinate of the centroid of detected edges
 * @param draw - whether or not to draw on image
 * @param top_color settings - YUV settings for the top color (black/blue)
 * @param bottom_color settings - YUV settings for the bottom color (green)
 * @return number of columns with edge transitions
 */
uint32_t find_edge_transitions(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                                uint8_t top_lum_min, uint8_t top_lum_max,
                                uint8_t top_cb_min, uint8_t top_cb_max,
                                uint8_t top_cr_min, uint8_t top_cr_max,
                                uint8_t bottom_lum_min, uint8_t bottom_lum_max,
                                uint8_t bottom_cb_min, uint8_t bottom_cb_max,
                                uint8_t bottom_cr_min, uint8_t bottom_cr_max)
{
  uint32_t columns_with_edge = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

  // Set scan region - either full frame or ROI
  uint16_t x_start = 0;
  uint16_t x_end = img->w;
  uint16_t y_start = 0;
  uint16_t y_end = img->h;
  
  // For camera 1 with ROI enabled, only scan the specified ROI
  if (use_roi1) {
    x_start = roi_x_min1;
    x_end = roi_x_max1;
    y_start = roi_y_min1;
    y_end = roi_y_max1;
    VERBOSE_PRINT("Using ROI for edge detection: x=%d-%d, y=%d-%d (%d columns)\n", 
                 x_start, x_end, y_start, y_end, (x_end-x_start));
  }

  // Check each column for transitions
  for (uint16_t x = x_start; x < x_end && x < img->w; x++) {
    bool edge_found = false;
    int edge_y = 0;
    
    // Check pairs of pixels in this column (looking for vertical transitions)
    for (uint16_t y = y_start; y < y_end-1 && y+1 < img->h; y++) {
      // Get YUV values for the current pixel and the one below it
      uint8_t *yp1, *up1, *vp1, *yp2, *up2, *vp2;
      
      // Get values for current pixel
      if (x % 2 == 0) {
        // Even x
        up1 = &buffer[y * 2 * img->w + 2 * x];      // U
        yp1 = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp1 = &buffer[y * 2 * img->w + 2 * x + 2];  // V
      } else {
        // Uneven x
        up1 = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        vp1 = &buffer[y * 2 * img->w + 2 * x];      // V
        yp1 = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
      
      // Get values for pixel below
      if (x % 2 == 0) {
        // Even x
        up2 = &buffer[(y+1) * 2 * img->w + 2 * x];      // U
        yp2 = &buffer[(y+1) * 2 * img->w + 2 * x + 1];  // Y1
        vp2 = &buffer[(y+1) * 2 * img->w + 2 * x + 2];  // V
      } else {
        // Uneven x
        up2 = &buffer[(y+1) * 2 * img->w + 2 * x - 2];  // U
        vp2 = &buffer[(y+1) * 2 * img->w + 2 * x];      // V
        yp2 = &buffer[(y+1) * 2 * img->w + 2 * x + 1];  // Y2
      }
      
      // Check for top_color -> bottom_color transition
      bool is_top_color = is_color_in_range(*yp1, *up1, *vp1, 
                                           top_lum_min, top_lum_max,
                                           top_cb_min, top_cb_max, 
                                           top_cr_min, top_cr_max);
                                           
      bool is_bottom_color = is_color_in_range(*yp2, *up2, *vp2, 
                                              bottom_lum_min, bottom_lum_max,
                                              bottom_cb_min, bottom_cb_max, 
                                              bottom_cr_min, bottom_cr_max);
      
      if (is_top_color && is_bottom_color) {
        // Found an edge transition in this column
        edge_found = true;
        edge_y = y;
        
        if (draw) {
          // Mark the edge in the image
          *yp1 = 255;  // Make pixel white
          *yp2 = 255;
        }
        
        // Just count one edge per column
        break;
      }
    }
    
    if (edge_found) {
      columns_with_edge++;
      tot_x += x;
      tot_y += edge_y;
    }
  }
  
  // Calculate centroid if any edges found
  if (columns_with_edge > 0) {
    *p_xc = (int32_t)roundf(tot_x / ((float) columns_with_edge) - img->w * 0.5f);
    *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) columns_with_edge));
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }
  
  // Draw ROI boundaries if in draw mode
  if (draw && use_roi1) {
    // Draw horizontal lines
    for (uint16_t x = roi_x_min1; x < roi_x_max1 && x < img->w; x++) {
      // Draw top ROI boundary
      if (roi_y_min1 < img->h) {
        uint8_t *yp = NULL;
        if (x % 2 == 0) {
          yp = &buffer[roi_y_min1 * 2 * img->w + 2 * x + 1];
        } else {
          yp = &buffer[roi_y_min1 * 2 * img->w + 2 * x + 1];
        }
        if (yp) *yp = 128; // Gray
      }
      
      // Draw bottom ROI boundary
      if (roi_y_max1-1 < img->h) {
        uint8_t *yp = NULL;
        if (x % 2 == 0) {
          yp = &buffer[(roi_y_max1-1) * 2 * img->w + 2 * x + 1];
        } else {
          yp = &buffer[(roi_y_max1-1) * 2 * img->w + 2 * x + 1];
        }
        if (yp) *yp = 128; // Gray
      }
    }
    
    // Draw vertical lines
    for (uint16_t y = roi_y_min1; y < roi_y_max1 && y < img->h; y++) {
      // Draw left ROI boundary
      if (roi_x_min1 < img->w) {
        uint8_t *yp = NULL;
        if (roi_x_min1 % 2 == 0) {
          yp = &buffer[y * 2 * img->w + 2 * roi_x_min1 + 1];
        } else {
          yp = &buffer[y * 2 * img->w + 2 * roi_x_min1 + 1];
        }
        if (yp) *yp = 128; // Gray
      }
      
      // Draw right ROI boundary
      if (roi_x_max1-1 < img->w) {
        uint8_t *yp = NULL;
        if ((roi_x_max1-1) % 2 == 0) {
          yp = &buffer[y * 2 * img->w + 2 * (roi_x_max1-1) + 1];
        } else {
          yp = &buffer[y * 2 * img->w + 2 * (roi_x_max1-1) + 1];
        }
        if (yp) *yp = 128; // Gray
      }
    }
  }
  
  return columns_with_edge;
}