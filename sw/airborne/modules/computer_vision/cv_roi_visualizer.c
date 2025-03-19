#include "modules/computer_vision/cv_roi_visualizer.h"
#include "modules/computer_vision/cv.h"
#include "lib/vision/image.h"
#include "modules/computer_vision/viewvideo.h" // For VIEWVIDEO_CAMERA (front_camera)

// ROI calculation parameters (must match settings in airframe file)
#define ROI_DIV_X 5
#define ROI_DIV_Y 3
#define ROI_CROP_X 2
#define ROI_CROP_Y 1

// Colors for visualization
static uint8_t roi_color[4] = {255, 0, 0, 255}; // Red color for ROI box

/**
 * Draw the ROI rectangle on the image
 * Function signature must match cv_function in cv.h
 */
static struct image_t *roi_visualizer(struct image_t *img, uint8_t camera_id)
{
  // Calculate ROI dimensions based on the grid system
  int cell_width = img->w / ROI_DIV_X;
  int cell_height = img->h / ROI_DIV_Y;
  
  // Calculate ROI coordinates (middle cell boundaries)
  int x_start = cell_width * ROI_CROP_X;
  int y_start = cell_height * ROI_CROP_Y;
  int x_end = img->w - cell_width * ROI_CROP_X;
  int y_end = img->h - cell_height * ROI_CROP_Y;
  
  // Draw the ROI rectangle
  struct point_t from, to;
  
  // Top horizontal line
  from.x = x_start; from.y = y_start;
  to.x = x_end; to.y = y_start;
  image_draw_line_color(img, &from, &to, roi_color);
  
  // Bottom horizontal line
  from.x = x_start; from.y = y_end;
  to.x = x_end; to.y = y_end;
  image_draw_line_color(img, &from, &to, roi_color);
  
  // Left vertical line
  from.x = x_start; from.y = y_start;
  to.x = x_start; to.y = y_end;
  image_draw_line_color(img, &from, &to, roi_color);
  
  // Right vertical line
  from.x = x_end; from.y = y_start;
  to.x = x_end; to.y = y_end;
  image_draw_line_color(img, &from, &to, roi_color);
  
  // Return the image with ROI visualization
  return img;
}

/**
 * Initialize the ROI visualizer
 */
void roi_visualizer_init(void)
{
  // Register the visualization callback on the front camera
  // cv_add_to_device takes a device, function pointer, fps, and ID
  cv_add_to_device(&VIEWVIDEO_CAMERA, roi_visualizer, 4, 0);
} 