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

#define OBJECT_DETECTOR_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS
#define COLOR_OBJECT_DETECTOR_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter settings
uint8_t green_lum_min = 0;
uint8_t green_lum_max = 0;
uint8_t green_cb_min = 0;
uint8_t green_cb_max = 0;
uint8_t green_cr_min = 0;
uint8_t green_cr_max = 0;
uint16_t green_threshold = 0;
uint16_t edge_threshold = 0;
uint8_t screen_fraction_scan = 0;
uint8_t downsample_factor = 1;

bool draw = true;
bool print_detections = false;

uint16_t edges_in_sector_1 = 0;
uint16_t edges_in_sector_2 = 0;
uint16_t edges_in_sector_3 = 0;

uint16_t green_in_sector_1 = 0;
uint16_t green_in_sector_2 = 0;
uint16_t green_in_sector_3 = 0;

// void apply_kernel(struct image_t *img, struct kernel *kernel, bool edge_detection, uint8_t weight);

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
};
struct color_object_t global_filters[2];


static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  // We still check the filter number if needed.
  // (In this example, we ignore 'draw' since we're applying a Gaussian blur.)
  switch (filter) {
    case 1:
      break;
    case 2:
      return img;
    default:
      return img;
  }

  // Define a local fixed 3x3 Gaussian kernel structure.
  // For a Gaussian blur, the kernel coefficients are:
  //   1  2  1
  //   2  4  2
  //   1  2  1
  // and they sum to 16 (the normalization factor).
  struct kernel3x3 {
    uint8_t size;      // 3 for a 3x3 kernel
    uint8_t boundary;  // 1 (one pixel border)
    int8_t values[9];  // Kernel coefficients (using signed values)
    // weight = 16
  };
  struct kernel5x5 {
    uint8_t size;      // 5 for a 5x5 kernel
    uint8_t boundary;  // 2 (two pixels border)
    int8_t values[25]; // Kernel coefficients (using signed values)
    // weight = 273
  };

  struct kernel3x3 kernel_3x3_gauss = {
    .size = 3,
    .boundary = 1,
    .values = { 1, 2, 1,
                2, 4, 2,
                1, 2, 1 }
  };

  struct kernel5x5 kernel_5x5_gauss = {
    .size = 5,
    .boundary = 2,
    .values = {
       1,  4,  7,  4,  1,
       4, 16, 26, 16,  4,
       7, 26, 41, 26,  7,
       4, 16, 26, 16,  4,
       1,  4,  7,  4,  1
    }
  };

  struct kernel5x5 kernel_5x5_sobel_hor = { //this is horizontal because image is transposed
    .size = 5,
    .boundary = 2, 
    .values = {
        -2, -1,  0,  1,  2,
        -3, -2,  0,  2,  3,
        -4, -3,  0,  3,  4,
        -3, -2,  0,  2,  3,
        -2, -1,  0,  1,  2
    }
  };

  struct kernel5x5 kernel_5x5_sobel_vert = {
    .size = 5,
    .boundary = 2, 
    .values = {
        -2, -3, -4, -3, -2,
        -1, -2, -3, -2, -1,
         0,  0,  0,  0,  0,
         1,  2,  3,  2,  1,
         2,  3,  4,  3,  2
    }
  };

  struct kernel5x5 kernel_5x5_sobel_vert_mirror = {
    .size = 5,
    .boundary = 2, 
    .values = {
        2, 3, 4, 3, 2,
        1, 2, 3, 2, 1,
        0,  0,  0,  0,  0,
       -1,  -2,  -3,  -2,  -1,
       -2,  -3,  -4,  -3,  -2
    }
  };


  edges_in_sector_1 = 0;
  edges_in_sector_2 = 0;
  edges_in_sector_3 = 0;

  green_in_sector_1 = 0;
  green_in_sector_2 = 0;
  green_in_sector_3 = 0;


  // Call apply_kernel to perform the convolution with a Gaussian filter.
  // edge_detection is false so that the_OBJECTs 16.
  // apply_kernel(img, (struct kernel *)&kernel_5x5_gauss, false, 273);
  // apply_kernel(img, (struct kernel *)&kernel_5x5_gauss, false, 350);
  // apply_kernel(img, (struct kernel *)&kernel_5x5_sobel_hor, true, 1);
  apply_kernel(img, (struct kernel *)&kernel_5x5_sobel_vert, true, 1, draw);
  // apply_kernel(img, (struct kernel *)&kernel_5x5_sobel_vert_mirror, true, 1);

  return img;
}


struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA, object_detector1, COLOR_OBJECT_DETECTOR_FPS, 0);



  green_lum_min = GREEN_DETECTOR_LUM_MIN;
  green_lum_max = GREEN_DETECTOR_LUM_MAX;
  green_cb_min = GREEN_DETECTOR_CB_MIN;
  green_cb_max = GREEN_DETECTOR_CB_MAX;
  green_cr_min = GREEN_DETECTOR_CR_MIN;
  green_cr_max = GREEN_DETECTOR_CR_MAX;
  green_threshold = GREEN_THRESHOLD;
  edge_threshold = EDGE_THRESHOLD;
  screen_fraction_scan = SCREEN_FRACTION_SCAN;
  downsample_factor = DOWNSAMPLE_FACTOR;
  print_detections = PRINT_DETECTIONS;


}

struct image_t deep_copy_image(struct image_t *img) {
  // Create a shallow copy of the image_t struct.
  struct image_t copy = *img;

  // Allocate memory for the new image buffer.
  copy.buf = malloc(img->buf_size);
  if (copy.buf == NULL) {
      return copy;  
  }

  // Copy the contents of the original buffer into the new buffer.
  memcpy(copy.buf, img->buf, img->buf_size);

  return copy;
}

void free_image(struct image_t *img) {
    if (img && img->buf) {
        free(img->buf);
        img->buf = NULL;  // Avoid dangling pointer
    }
}

struct kernel{
  uint8_t size; // how large is kernel (always square NxN) so for a 3x3 size=3
  uint8_t boundary; // how much boundary do we have (for 3, its 1 pixel, 5 gives 2 pixels etc.)
  int8_t values[];
};

void apply_kernel(struct image_t *img, struct kernel *kernel, bool edge_detection, uint8_t weight, bool draw) {
  // Create a static copy of the original image buffer for detection.
  struct image_t static_copy = deep_copy_image(img);
  if (!static_copy.buf) {
    return;
  }

  uint8_t *buffer = (uint8_t *)img->buf;
  uint8_t *static_buffer = (uint8_t *)static_copy.buf;
  uint8_t boundary = kernel->boundary;

  // Loop over each pixel 
  for (uint16_t y = boundary; y < img->h - boundary; y += downsample_factor) {
    for (uint16_t x = boundary; x < img->w - boundary - screen_fraction_scan * img->w / 20; x += downsample_factor) {
      uint8_t *yp, *up, *vp;
      int16_t result = 0;
      uint8_t kernel_index = 0;

      // Compute pointers for Y, U, V from the live buffer.
      if (x % 2 == 0) {
        // Even x: U, Y, V sequence.
        up = &buffer[y * 2 * img->w + 2 * x];        // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];      // Y
        vp = &buffer[y * 2 * img->w + 2 * x + 2];      // V
      } else {
        // Odd x.
        up = &buffer[y * 2 * img->w + 2 * x - 2];      // U
        vp = &buffer[y * 2 * img->w + 2 * x];          // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];      // Y
      }

      // Convolution: iterate over the kernel window using the original static copy.
      for (int8_t row = -((int8_t)kernel->boundary); row <= (int8_t)kernel->boundary; row++) {
        for (int8_t col = -((int8_t)kernel->boundary); col <= (int8_t)kernel->boundary; col++) {
          result += kernel->values[kernel_index] *
                    static_buffer[(y + row) * 2 * img->w + 2 * x + 2 * col + 1];
          kernel_index++;
        }
      }

      // Use the original (static) buffer for color detection.
      uint8_t orig_y, orig_up, orig_vp;
      if (x % 2 == 0) {
        orig_up = static_buffer[y * 2 * img->w + 2 * x];
        orig_y  = static_buffer[y * 2 * img->w + 2 * x + 1];
        orig_vp = static_buffer[y * 2 * img->w + 2 * x + 2];
      } else {
        orig_up = static_buffer[y * 2 * img->w + 2 * x - 2];
        orig_vp = static_buffer[y * 2 * img->w + 2 * x];
        orig_y  = static_buffer[y * 2 * img->w + 2 * x + 1];
      }

      bool isgreen = (orig_y > green_lum_min && orig_y < green_lum_max &&
                      orig_up > green_cb_min && orig_up < green_cb_max &&
                      orig_vp > green_cr_min && orig_vp < green_cr_max);

      if (isgreen) {
        // Partition into sectors based on y (as before).
        if (y < img->h / 3) {
          green_in_sector_1++;
          if (draw) {
            *yp = 200;  // Draw red (sector 1)
            *up = 84;
            *vp = 255;
            if (result > edge_threshold) {
              edges_in_sector_1++;
              *yp = 76;
              *up = 100;
              *vp = 90;
            }
          } else {
            if (result > edge_threshold)
              edges_in_sector_1++;
          }
        } else if (y < img->h * 2 / 3) {
          green_in_sector_2++;
          if (draw) {
            *yp = 200;  // Draw green (sector 2)
            *up = 43;
            *vp = 21;
            if (result > edge_threshold) {
              edges_in_sector_2++;
              *yp = 149;
              *up = 100;
              *vp = 60;
            }
          } else {
            if (result > edge_threshold)
              edges_in_sector_2++;
          }
        } else {
          green_in_sector_3++;
          if (draw) {
            *yp = 200;  // Draw blue (sector 3)
            *up = 255;
            *vp = 107;
            if (result > edge_threshold) {
              edges_in_sector_3++;
              *yp = 29;
              *up = 100;
              *vp = 150;
            }
          } else {
            if (result > edge_threshold)
              edges_in_sector_3++;
          }
        }
      }
    }
  }

  if (print_detections) {
    VERBOSE_PRINT("Green in sector 1,2,3: (%d, %d, %d)\n", 
      green_in_sector_1, green_in_sector_2, green_in_sector_3);

    VERBOSE_PRINT("Edges in sector 1,2,3: (%d, %d, %d)\n", edges_in_sector_1, edges_in_sector_2, edges_in_sector_3);
  }


  // uint16_t totalarea = ((img->h - 2 * boundary) / 3) * (img->w - boundary - screen_fraction_scan * img->w / 20);
  // VERBOSE_PRINT("Total pixels per area: (%d)", totalarea);

  free_image(&static_copy);
  return;
}



void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  AbiSendMsgGREEN_DETECTION_GROUP_11(GREEN_DETECTION_GROUP_11_ID, green_in_sector_1, green_in_sector_2, green_in_sector_3);
  AbiSendMsgEDGE_DETECTION_GROUP_11(EDGE_DETECTION_GROUP_11_ID, edges_in_sector_1, edges_in_sector_2, edges_in_sector_3);

}
