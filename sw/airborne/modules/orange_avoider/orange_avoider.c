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

enum navigation_state_t {
  SAFE,
  SEARCH_FOR_BEST_HEADING,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
  };

// define settings
float oa_color_count_frac = 0.4f; // this is the only one used right now for thresholds
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;

float heading_increment = 5.f;          // heading angle increment [deg]
float heading_change_best = 5.f;
float maxDistance = 2.25f;               // max waypoint displacement [m]
uint8_t minimum_center_confidence_for_move = 2; //When trying to move left or right if center confidence is lower than this number, drone wont move.

const int16_t max_trajectory_confidence = 10; // number of consecutive negative object detections to be sure we are obstacle free

int16_t green_pixels_sector_1_cb = 0; 
int16_t green_pixels_sector_2_cb = 0;
int16_t green_pixels_sector_3_cb = 0;

int32_t reward_left = 0;
int32_t reward_center = 0;
int32_t reward_right = 0;

int8_t left_confidence = 0;
int8_t center_confidence = 0;
int8_t right_confidence = 0;

#ifndef GREEN_DETECTION_GROUP_11_ID
#define GREEN_DETECTION_GROUP_11_ID ABI_BROADCAST
#endif
static abi_event green_detection_ev; 
static void green_detection_cb(uint8_t __attribute__((unused)) sender_id, 
                            int16_t green_pixels_sector_1, 
                            int16_t green_pixels_sector_2,
                            int16_t green_pixels_sector_3
                            ) {
  green_pixels_sector_1_cb = green_pixels_sector_1;
  green_pixels_sector_2_cb = green_pixels_sector_2;
  green_pixels_sector_3_cb = green_pixels_sector_3;
}


uint16_t edge_count_sector_1_cb = 0;
uint16_t edge_count_sector_2_cb = 0;
uint16_t edge_count_sector_3_cb = 0;

#ifndef EDGE_DETECTION_GROUP_11_ID
#define EDGE_DETECTION_GROUP_11_ID ABI_BROADCAST
#endif
static abi_event edge_detection_ev; 
static void edge_count_cb(uint8_t __attribute__((unused)) sender_id, uint16_t edge_count_sector_1, uint16_t edge_count_sector_2, uint16_t edge_count_sector_3) {
  edge_count_sector_1_cb = edge_count_sector_1;
  edge_count_sector_2_cb = edge_count_sector_2;
  edge_count_sector_3_cb = edge_count_sector_3;
}


/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void orange_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our green detection and edge detection callbacks
  AbiBindMsgGREEN_DETECTION_GROUP_11(GREEN_DETECTION_GROUP_11_ID, &green_detection_ev, green_detection_cb);
  AbiBindMsgEDGE_DETECTION_GROUP_11(EDGE_DETECTION_GROUP_11_ID, &edge_detection_ev, edge_count_cb);
}

void orange_avoider_periodic(void)
{
    // Static counters to build up forward movement and to avoid endless search turning.
    static int safe_counter = 0;
    static int search_counter = 0;

    // Only execute if we are flying.
    if (!autopilot_in_flight()) {
        return;
    }

    // Compute the pixel area per sector of the front camera.
    int32_t area = (front_camera.output_size.w / 3) * (front_camera.output_size.h / 2);
    int32_t minimum_reward = (int32_t)(oa_color_count_frac * area);

    // Get the green pixel counts for each sector.
    int32_t reward_green_left   = 2 * green_pixels_sector_1_cb;
    int32_t reward_green_center = green_pixels_sector_2_cb;
    int32_t reward_green_right  = 2 * green_pixels_sector_3_cb;

    // Get edge counts for each sector
    int32_t reward_edge_left = edge_count_sector_1_cb;
    int32_t reward_edge_center = edge_count_sector_2_cb;
    int32_t reward_edge_right = edge_count_sector_3_cb;   
    
    int edge_weight = 25; 
    
    // Adjust these reward functions to change sensitivity to edges or green pixels etc....
    reward_left = reward_green_left - reward_edge_left * edge_weight;
    reward_center = reward_green_center - reward_edge_center * edge_weight;
    reward_right = reward_green_right - reward_edge_right * edge_weight;

    // Update obstacle detection confidences.
    if (reward_center > minimum_reward) {
        center_confidence++;
    } else {
        center_confidence -= 2;
    }

    if (reward_right > minimum_reward) {
      right_confidence++;
    } else {
        right_confidence -= 2;
    }

    if (reward_left > minimum_reward) {
      left_confidence++;
    } else {
        left_confidence -= 2;
    }
    // bound confidences
    Bound(center_confidence, 0, max_trajectory_confidence);
    Bound(left_confidence, 0, max_trajectory_confidence);
    Bound(right_confidence, 0, max_trajectory_confidence);

    float moveDistance = fminf(maxDistance, 0.2f * center_confidence);

    switch (navigation_state) {
      case SAFE:
        VERBOSE_PRINT("State: SAFE. Confidence: L=%d, C=%d, R=%d; Reward: L=%d, C=%d, R=%d\n",
                      left_confidence, center_confidence, right_confidence,
                      reward_left, reward_center, reward_right);
        
        // Move waypoint forward
        moveWaypointForward(WP_TRAJECTORY, moveDistance);
        
        // Check if inside obstacle zone
        if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
          navigation_state = OUT_OF_BOUNDS;
        }
        // If center is not safe, verify whether left or right is safe
        else if (reward_center < minimum_reward) {
          if ((reward_left > minimum_reward) || (reward_right > minimum_reward)) {
            navigation_state = SEARCH_FOR_BEST_HEADING;
          } else {
            navigation_state = SEARCH_FOR_SAFE_HEADING;
          }
        } 
        else { 
          // Regular move, varies by confidence score (moveDistance is defined that way)
          moveWaypointForward(WP_GOAL, moveDistance);
          moveWaypointForward(WP_RETREAT, -1.0f * moveDistance);
        }
        break;
    
      case SEARCH_FOR_BEST_HEADING:
        VERBOSE_PRINT("State: SEARCH_FOR_BEST_HEADING. Confidence: L=%d, C=%d, R=%d; Reward: L=%d, C=%d, R=%d\n",
                      left_confidence, center_confidence, right_confidence,
                      reward_left, reward_center, reward_right);
        
        // Decide if left or right has more green.
        if (reward_left > minimum_reward) {
          increase_nav_heading(-(left_confidence - center_confidence) * heading_change_best);
          if (center_confidence >= minimum_center_confidence_for_move) {
            navigation_state = SAFE;
            break;
          } 
        }
        else if (reward_right > minimum_reward) {
          increase_nav_heading((right_confidence - center_confidence) * heading_change_best);
          if (center_confidence >= minimum_center_confidence_for_move) {
            navigation_state = SAFE;
            break;
          }
        } 
        else {
          navigation_state = SEARCH_FOR_SAFE_HEADING;
          break;
        }
        break;
    
      case SEARCH_FOR_SAFE_HEADING:
        VERBOSE_PRINT("State: SEARCH_FOR_SAFE_HEADING. Confidence: L=%d, C=%d, R=%d; Reward: L=%d, C=%d, R=%d\n",
                      left_confidence, center_confidence, right_confidence,
                      reward_left, reward_center, reward_right);
        // Stop
        waypoint_move_here_2d(WP_GOAL);
        waypoint_move_here_2d(WP_RETREAT);
        waypoint_move_here_2d(WP_TRAJECTORY);
        
        waypoint_move_here_2d(WP_GOAL);
        waypoint_move_here_2d(WP_RETREAT);
        waypoint_move_here_2d(WP_TRAJECTORY);
      

        if (left_confidence == 0 && right_confidence == 0) {
          // Turn far left if no confidence on either side.
          increase_nav_heading(-3*heading_change_best);
        }
        else if (left_confidence >= right_confidence) {
          // Turn left
          increase_nav_heading(-(left_confidence - center_confidence)*heading_change_best); // Turn harder if difference in confidence is large
          VERBOSE_PRINT("Turning left, confidence: L=%d, R=%d\n", left_confidence, right_confidence);
          if (center_confidence >= minimum_center_confidence_for_move) {
            navigation_state = SAFE;
            break;
          }
        }
        else {
          // Turn right
          increase_nav_heading((right_confidence - center_confidence)*heading_change_best);  // Turn harder if difference in confidence is large
          VERBOSE_PRINT("Turning right, confidence: L=%d, R=%d\n", left_confidence, right_confidence);
          if (center_confidence >= minimum_center_confidence_for_move) {
            navigation_state = SAFE;
            break;
          }
        }
        // Make sure we have a couple of good readings before declaring the way safe.
        if (center_confidence >= minimum_center_confidence_for_move) {
          navigation_state = SAFE;
        }
        break;
    
      case OUT_OF_BOUNDS:
        VERBOSE_PRINT("State: OUT_OF_BOUNDS. Confidence: L=%d, C=%d, R=%d; Reward: L=%d, C=%d, R=%d\n",
                      left_confidence, center_confidence, right_confidence,
                      reward_left, reward_center, reward_right);
        
        increase_nav_heading(heading_increment);
        moveWaypointForward(WP_TRAJECTORY, 1.5f);
        moveWaypointForward(WP_RETREAT, -1.0f);
      
        if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
          // Add offset to head back into arena
          increase_nav_heading(heading_increment);
          // Reset safe counter (here, we reset center confidence)
          center_confidence = 0;
          // Ensure direction is safe before continuing
          navigation_state = SEARCH_FOR_SAFE_HEADING;
        }
        break;
    
      default:
        VERBOSE_PRINT("State: DEFAULT. Confidence: L=%d, C=%d, R=%d; Reward: L=%d, C=%d, R=%d\n",
                      left_confidence, center_confidence, right_confidence,
                      reward_left, reward_center, reward_right);
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

  // VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
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
  // VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
  //               POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
  //               stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  // VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
  //               POS_FLOAT_OF_BFP(new_coor->y));
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

