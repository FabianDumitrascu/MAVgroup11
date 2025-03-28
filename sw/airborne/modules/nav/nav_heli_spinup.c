/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/nav/nav_heli_spinup.c"
 * @author Freek van Tienen
 * This module controls the spinup of the main rotor from a Helicopter
 */

#include "modules/nav/nav_heli_spinup.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "navigation.h"
#include "paparazzi.h"

static struct nav_heli_spinup_t nav_heli_spinup;

/**
 * @param duration duration over which the heli should spin up [s]
 * @param throttle the final throttle at the end of the spinup
 *
 * Sets the heli spinup routine parameters and sets the horizontal and
 * vertical modes to manual, with zero commands
 */
void nav_heli_spinup_setup(uint16_t duration, float throttle)
{
  nav_heli_spinup.duration = (duration > 0) ? duration : 1;
  nav_heli_spinup.throttle = throttle * MAX_PPRZ;

#ifdef COMMAND_ROLL
  stabilization_cmd[COMMAND_ROLL]  = 0;
#endif
#ifdef COMMAND_PITCH
  stabilization_cmd[COMMAND_PITCH] = 0;
#endif
#ifdef COMMAND_YAW
  stabilization_cmd[COMMAND_YAW]   = 0;
#endif
  nav.throttle = 0;

  nav.horizontal_mode = NAV_HORIZONTAL_MODE_NONE;
  nav.vertical_mode = NAV_VERTICAL_MODE_MANUAL;
}

/**
 * Runs the heli spinup routine, with the parameters set by
 * nav_heli_spinup_setup
 */
bool nav_heli_spinup_run(void)
{
  if (stage_time > nav_heli_spinup.duration) {
    return false;
  }

#ifdef COMMAND_ROLL
  stabilization_cmd[COMMAND_ROLL]  = 0;
#endif
#ifdef COMMAND_PITCH
  stabilization_cmd[COMMAND_PITCH] = 0;
#endif
#ifdef COMMAND_YAW
  stabilization_cmd[COMMAND_YAW]   = 0;
#endif
  nav.throttle = stage_time * nav_heli_spinup.throttle / nav_heli_spinup.duration;

  nav.horizontal_mode = NAV_HORIZONTAL_MODE_NONE;
  nav.vertical_mode = NAV_VERTICAL_MODE_MANUAL;
  return true;
}
