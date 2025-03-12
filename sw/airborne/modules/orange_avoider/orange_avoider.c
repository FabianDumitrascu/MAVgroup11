/*
* Copyright (C) Roland Meertens
*
* This file is part of paparazzi
*
*/
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * Minimal program that sets the drone's heading to 45 degrees.
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include <math.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE FALSE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Global variable for the desired starting heading (in degrees)
float startHeading = 10.0f;
float oa_color_count_frac = 0.18f;

uint8_t chooseStartPath(float startHeading); 
uint8_t verifyHeading(float goalHeading);



/*
* Initialization function.
*/
void orange_avoider_init(void) {
  srand(time(NULL));
}

/*
* Periodic function.
* In this minimal example, it only logs the current heading if the autopilot is in flight.
*/
void orange_avoider_periodic(void) {

  if (!autopilot_in_flight()) {
  VERBOSE_PRINT("Drone not in Flight.");
  return;
  }
  VERBOSE_PRINT("Verify heading output: %i\n", verifyHeading(startHeading));

  if (verifyHeading(startHeading) != 1){
    chooseStartPath(startHeading);
  }

  VERBOSE_PRINT("Periodic: current heading is %f\n", DegOfRad(stateGetNedToBodyEulers_f()->psi));
}

/*
* Sets the drone's heading based on the provided startHeading (in degrees)
*/
uint8_t chooseStartPath(float startHeading) {
  // Convert startHeading from degrees to radians and store in new_heading.
  float new_heading = RadOfDeg(startHeading);

  // Normalize new_heading to the range [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // Set the drone's heading
  nav.heading = new_heading;

  VERBOSE_PRINT("Setting heading to %f\n", DegOfRad(new_heading));
  return 0;
}

uint8_t verifyHeading(float goalHeading) {
  float currentHeading, difference, maxError;
  maxError = 1.0f;
  currentHeading = DegOfRad(nav.heading);
  difference = fabsf(currentHeading - goalHeading);
  VERBOSE_PRINT("currentHeading=%f, goalHeading=%f, difference=%f", currentHeading, goalHeading, difference);
  if (difference < maxError){
    return 1;
  }
  return 0;
}

 