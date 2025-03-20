/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);

// Function prototype
void reset_confidence_scores(void);
enum navigation_state_t {
  SAFE,
  OBSTACLE_FAR,
  SEARCH_FOR_BEST_HEADING,
  OBSTACLE_NEAR,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
  };

// define settings
float oa_color_count_frac = 0.1f;
float obstacle_far_count_frac = 0.3f;

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int16_t center_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float heading_increment = 5.f;          // heading angle increment [deg]
float maxDistance = 3;               // max waypoint displacement [m]

const int16_t max_trajectory_confidence = 10; // number of consecutive negative object detections to be sure we are obstacle free

/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */

int16_t pixel_count_center = 0;
int16_t pixel_count_left = 0;
int16_t pixel_count_right = 0;

int16_t left_confidence = 0;
int16_t right_confidence = 0;

int32_t cost_left = 0;
int32_t cost_center = 0;
int32_t cost_right = 0;

#ifndef TEST_DETECTION
#define TEST_DETECTION ABI_BROADCAST
#endif
static abi_event segmented_counter_ev;
static void segmented_counter_cb(uint8_t __attribute__((unused)) sender_id,
			  int16_t count_left, int16_t count_center, int16_t count_right)
{
  pixel_count_left = count_left;
  pixel_count_center = count_center;
  pixel_count_right = count_right;
}

#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif


void reset_confidence_scores(){
  left_confidence = 0;
  right_confidence = 0;
  center_confidence= 0;
  // max_trajectory_confidence = 0;
}

static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}
  
  

/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void orange_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgSEGMENTED_DETECTION(SEGMENTED_DETECTION_ID, &segmented_counter_ev, segmented_counter_cb);
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
 
void orange_avoider_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  // compute current cost threshold
  int32_t near_cost_threshold = oa_color_count_frac * (front_camera.output_size.w/3) * (front_camera.output_size.h/2);
  int32_t far_cost_threshold = obstacle_far_count_frac * (front_camera.output_size.w/3) * (front_camera.output_size.h/2);

  // Define the cost function based on green pixel count and number of edges here:
  int32_t cost_left = pixel_count_left;
  int32_t cost_center = pixel_count_center;
  int32_t cost_right = pixel_count_right;

  //VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  VERBOSE_PRINT("Current Confidence left: %d, center: %d, right: %d, state: %d", left_confidence, center_confidence, right_confidence, navigation_state);
  // update our safe confidence using color threshold
  if(cost_center > far_cost_threshold){
    center_confidence++;
  } else {
    center_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound center_confidence
  Bound(center_confidence, 0, max_trajectory_confidence);

  
  if ( (cost_left < near_cost_threshold) && (cost_right < near_cost_threshold) ){
    left_confidence-=2;
    right_confidence-=2;
    
  } else if (cost_left >= cost_right){
       left_confidence ++;
       right_confidence-=2;
  } else {
       left_confidence -=2;
       right_confidence++;
  }

  // bound confidences
  Bound(center_confidence, 0, max_trajectory_confidence);
  Bound(left_confidence, 0, max_trajectory_confidence);
  Bound(right_confidence, 0, max_trajectory_confidence);
  
  float moveDistance = fminf(maxDistance, 0.2f * center_confidence);

  switch (navigation_state){
    case SAFE:
    
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else if (cost_center < far_cost_threshold){
        navigation_state = OBSTACLE_FAR;
      } else if (center_confidence == 0){
        navigation_state = OBSTACLE_NEAR;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
        moveWaypointForward(WP_RETREAT, -1.0f * moveDistance);
      }
      break;
    case OBSTACLE_FAR:
      // Only print for now
      VERBOSE_PRINT("There is an obstacle far away");
      navigation_state = SEARCH_FOR_BEST_HEADING;

      break;
    case SEARCH_FOR_BEST_HEADING:
      // Decide if left or right has more green.
      if ( (left_confidence >= right_confidence) &&
          (cost_left >= far_cost_threshold) ){
        increase_nav_heading(-30.f);
        if (center_confidence >=2){
           navigation_state = SAFE;
           break;
        }
      } else if (cost_right >= far_cost_threshold){
         increase_nav_heading(30.f);
         if (center_confidence >=2){
           navigation_state = SAFE;
           break;
         }
      } else if (center_confidence >= 2){
         navigation_state = SAFE;
         break;
      } else {
        navigation_state = OBSTACLE_NEAR;
        break;
      }

      break; 
    case OBSTACLE_NEAR:
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_RETREAT);
      waypoint_move_here_2d(WP_TRAJECTORY);

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      if (left_confidence == 0 && right_confidence ==0)
      {
        // Turn far left
        increase_nav_heading(-90.f);
      }
      else if (left_confidence > right_confidence) {
        // Turn left
        increase_nav_heading(-30.f); // Turn 30 degrees left
        VERBOSE_PRINT("Turning left, confidence: L=%d, R=%d\n", left_confidence, right_confidence);
        if (center_confidence >=2){
           navigation_state = SAFE;
           break;
        }
      } else {
        // Turn right
        increase_nav_heading(30.f); // Turn 30 degrees right
        VERBOSE_PRINT("Turning right, confidence: L=%d, R=%d\n", left_confidence, right_confidence);
        if (center_confidence >=2){
           navigation_state = SAFE;
           break;
        }
      }
      // make sure we have a couple of good readings before declaring the way safe
      if (center_confidence >= 2){
        navigation_state = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);
      moveWaypointForward(WP_RETREAT, -1.0f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(heading_increment);

        // reset safe counter
        center_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;
    default:
      break;
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  //VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  //VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
    //            POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
      //          stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  //VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
    //            POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = 5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}

