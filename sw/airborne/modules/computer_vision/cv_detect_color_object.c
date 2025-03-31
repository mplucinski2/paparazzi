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

bool cod_draw1 = false;

//region of interest limits
uint16_t roi_x_min1 = 0; 
uint16_t roi_x_max1 = 60;
uint16_t roi_y_min1 = 240;
uint16_t roi_y_max1 = 280;
bool use_roi1 = true;

struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  uint32_t roi_area;  //region of interest area
  bool updated;
};
struct color_object_t global_filters[1]; 

uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);

/*
 * object_detector
 * @param img - input image to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t __attribute__((unused)) filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;

  lum_min = cod_lum_min1;
  lum_max = cod_lum_max1;
  cb_min = cod_cb_min1;
  cb_max = cod_cb_max1;
  cr_min = cod_cr_min1;
  cr_max = cod_cr_max1;
  draw = cod_draw1;

  int32_t x_c, y_c;
  uint32_t roi_area = 0;

  uint32_t count = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
  
  //total number of pixels in the region of interest
  if (use_roi1) {
    roi_area = (roi_x_max1 - roi_x_min1) * (roi_y_max1 - roi_y_min1);
  } else {
    roi_area = img->w * img->h;
  }
  
  //print to verify the fraction of green pixels and the total number of pixels in the region of interest
  VERBOSE_PRINT("found %d green pixels in %s (ROI area: %d pixels)\n", 
               count, use_roi1 ? "ROI" : "full frame", roi_area);

  pthread_mutex_lock(&mutex);
  global_filters[0].color_count = count;
  global_filters[0].x_c = x_c;
  global_filters[0].y_c = y_c;
  global_filters[0].roi_area = roi_area; 
  global_filters[0].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

//region of interest is symmetrical about the vertical axis, y_max is just a function of y_min
void update_roi_y_max(void) {
  roi_y_max1 = 520 - roi_y_min1;
  VERBOSE_PRINT("opdated ROI coords: y_min=%d, y_max=%d\n", roi_y_min1, roi_y_max1);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, sizeof(struct color_object_t));
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

  //initializing roi boundaries from defines in the xml
#ifdef COLOR_OBJECT_DETECTOR_ROI_X_MIN1
  roi_x_min1 = COLOR_OBJECT_DETECTOR_ROI_X_MIN1;
  roi_x_max1 = COLOR_OBJECT_DETECTOR_ROI_X_MAX1;
  roi_y_min1 = COLOR_OBJECT_DETECTOR_ROI_Y_MIN1;
  update_roi_y_max();  
  use_roi1 = true;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
#endif
}

/*
 * find_object_centroid
 *
 * finds the centroid of pixels in an image within roi bounds.
 * Also returns the amount of pixels that satisfy these filter yuv thresholds.
 * Uses front camera with roi for detection.
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
  
  //setting the region of interest
  uint16_t x_start = 0;
  uint16_t x_end = img->w;
  uint16_t y_start = 0;
  uint16_t y_end = img->h;
  
  //applying roi if roi is set to true
  if (use_roi1) {
    x_start = roi_x_min1;
    x_end = roi_x_max1;
    y_start = roi_y_min1;
    y_end = roi_y_max1;
    VERBOSE_PRINT("using roifor camera: x=%d-%d, y=%d-%d (%d pixels total)\n", 
                  x_start, x_end, y_start, y_end, (x_end-x_start)*(y_end-y_start));
  }

  //only looping for pixels in the region of interest
  for (uint16_t y = y_start; y < y_end && y < img->h; y++) {
    for (uint16_t x = x_start; x < x_end && x < img->w; x ++) {
      //checking if color fits in the specified bounds
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        //even width x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
      } else {
        //uneven width x
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
          *yp = 255;  
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
  
  //printing debug message
  static uint8_t debug_counter = 0;
  if (debug_counter++ % 50 == 0) {
    VERBOSE_PRINT("Detected %d green pixels out of %d in ROI (%d%%)\n", 
                 cnt, (x_end-x_start)*(y_end-y_start), (int)(100.0*cnt/((x_end-x_start)*(y_end-y_start))));
  }
  
  return cnt;
}

void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[1];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
    //using extra field to send roi info
    int16_t roi_area_scaled = (int16_t)(local_filters[0].roi_area > 32767 ? 32767 : local_filters[0].roi_area);
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        0, 0, local_filters[0].color_count, roi_area_scaled);
    local_filters[0].updated = false;
  }
}

/**
 *handler function for setting ROI Y Min
 */
void cv_detect_color_object_SetRoiYMin(float val)
{
  roi_y_min1 = (uint16_t)val;
  update_roi_y_max();
}