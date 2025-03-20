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

#define OBJECT_DETECTOR_VERBOSE FALSE

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

uint8_t edge_lum_min = 0;
uint8_t edge_lum_max = 0;
uint8_t edge_cb_min = 0;
uint8_t edge_cb_max = 0;
uint8_t edge_cr_min = 0;
uint8_t edge_cr_max = 0;
uint8_t edge_threshold = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;

uint16_t edges_in_sector_1 = 0;
uint16_t edges_in_sector_2 = 0;
uint16_t edges_in_sector_3 = 0;

uint16_t green_in_sector_1 = 0;
uint16_t green_in_sector_2 = 0;
uint16_t green_in_sector_3 = 0;

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
      break;
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
  apply_kernel(img, (struct kernel *)&kernel_5x5_sobel_vert, true, 1);
  apply_kernel(img, (struct kernel *)&kernel_5x5_sobel_vert_mirror, true, 1);

  // VERBOSE_PRINT("Applied Gaussian filter convolution\n");

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

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2, 1);
#endif


// initialize edge detection variables
#ifdef EDGE_DETECTOR_LUM_MIN
  edge_lum_min = EDGE_DETECTOR_LUM_MIN;
  edge_lum_max = EDGE_DETECTOR_LUM_MAX;
  edge_cb_min = EDGE_DETECTOR_CB_MIN;
  edge_cb_max = EDGE_DETECTOR_CB_MAX;
  edge_cr_min = EDGE_DETECTOR_CR_MIN;
  edge_cr_max = EDGE_DETECTOR_CR_MAX;
  edge_threshold = EDGE_DETECTOR_THRESHOLD;
#endif

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

void apply_kernel(struct image_t *img, struct kernel *kernel, bool edge_detection, uint8_t weight) {
  struct image_t static_copy = deep_copy_image(img);
  if (!static_copy.buf) {
    return;
  }

  uint8_t *buffer = (uint8_t *)img->buf;
  uint8_t *static_buffer = (uint8_t *)static_copy.buf;
  uint8_t boundary = kernel->boundary;


  for (uint16_t y = boundary; y < img->h - boundary; y++) {
    for (uint16_t x = boundary; x < img->w - boundary; x++) {
      uint8_t *yp, *up, *vp;
      int16_t result = 0;
      uint8_t kernel_index = 0;

      // Determine the Y channel pointer (assuming Y is at offset +1 for even x)
      yp = &buffer[y * 2 * img->w + 2 * x + 1];

      // Set U and V pointers based on even/odd x positioning in YUV422
      if (x % 2 == 0) {
        up = &buffer[y * 2 * img->w + 2 * x];
        vp = &buffer[y * 2 * img->w + 2 * x + 2];
      } else {
        up = &buffer[y * 2 * img->w + 2 * x - 2];
        vp = &buffer[y * 2 * img->w + 2 * x];
      }

      // Convolution: iterate over the kernel window
      for (int8_t row = -((int8_t)kernel->boundary); row <= (int8_t)kernel->boundary; row++) {
        for (int8_t col = -((int8_t)kernel->boundary); col <= (int8_t)kernel->boundary; col++) {
          result += kernel->values[kernel_index] *
                    static_buffer[(y + row) * 2 * img->w + 2 * x + 2 * col + 1];
          // VERBOSE_PRINT("kernel_value = %d, static_buffer = %d, index = %d\n", 
          //   (int8_t)kernel->values[kernel_index], 
          //   static_buffer[(y + row) * 2 * img->w + 2 * x + 2 * col + 1],
          //   kernel_index);
          kernel_index++;
        }
      }

      if (edge_detection) {
        // For edge detection, if the convolution result exceeds the threshold, mark the pixel in pink.
        bool isgreen = (*yp > cod_lum_min1 && *yp < cod_lum_max1 &&
                        *up > cod_cb_min1 && *up < cod_cb_max1 &&
                        *vp > cod_cr_min1 && *vp < cod_cr_max1);
        if (isgreen) {
          if (y < img->h / 3){
            green_in_sector_1++;
            *yp = 200;  // Y channel
            *up = 84;  // U channel
            *vp = 255;  // V channel
            if (result > edge_threshold) {
              edges_in_sector_1++;
              *yp = 76;  // Y channel
              *up = 100;  // U channel
              *vp = 90;  // V channel
            }
          }
          if (y > img->h / 3 && y < img->h * 2 / 3){
            green_in_sector_2++;
            *yp = 200;  // Y channel
            *up = 43;  // U channel
            *vp = 21;  // V channel
            if (result > edge_threshold) {
              edges_in_sector_2++;
              *yp = 149;  // Y channel
              *up = 100;  // U channel
              *vp = 60;  // V channel
            }
          }
          if (y > img->h * 2/ 3){
            green_in_sector_3++;
            *yp = 200;  // Y channel
            *up = 255;  // U channel
            *vp = 107;  // V channel
            if (result > edge_threshold) {
              edges_in_sector_3++;
              *yp = 29;  // Y channel
              *up = 100;  // U channel
              *vp = 150;  // V channel
            }
          }
        }

      } else {
        // For a Gaussian blur, replace the luminance with the normalized convolution result.
        // Here, 'weight' should be the sum of the kernel weights (to normalize the average).
        *yp = result / weight;
      }
    }
  }

  
  VERBOSE_PRINT("Green in sector 1,2,3: (%d, %d, %d)\n", green_in_sector_1, green_in_sector_2, green_in_sector_3);

  free_image(&static_copy);
  return;
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

  AbiSendMsgGREEN_DETECTION_GROUP_11(GREEN_DETECTION_GROUP_11_ID, green_in_sector_1, green_in_sector_2, green_in_sector_3);
  AbiSendMsgEDGE_DETECTION_GROUP_11(EDGE_DETECTION_GROUP_11_ID, edges_in_sector_1, edges_in_sector_2, edges_in_sector_3);

}
