/*
 * Copyright (C) 2022 Jonathas Laffita <jonathaslvdh@gmail.com>
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

/** @file "modules/airborne_docking/airborne_docking_overactuated.c"
 * @author Jonathas Laffita <jonathaslvdh@gmail.com>
 * Airborne docking module for relative setpoint formulation and acceleration command
 */


#include "modules/airborne_docking/airborne_docking_overactuated.h"
#include "generated/airframe.h"         ///  NEEDED??
#include "modules/core/abi.h"
#include <stdio.h>                      ///NEEDED?     
#include <time.h>                       ///NEEDED?



/// MY DEFINES
#ifndef QWE_ID 
#define QWE_ID ABI_BROADCAST
#endif


enum Airborne_guidance_state_t {                         //WHAT CAN I USE THIS FOR?
  LOOKING_MANUAL,
  TARGET_FOUND_PRECONTACT_SETPOINT,
  TERMINAL_GUIDANCE,
};

// define settings
float oag_color_count_frac = 0.18f;       // Relative position GAIN
float oag_floor_count_frac = 0.05f;       // relative velocity GAIN
float oag_max_speed = 0.5f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]










static abi_event qwe_ev;

static void qwe_cb(uint8_t sender_id, uint8_t flag, struct FloatVect3 * accel_sp)
{
  // your abi callback code here
}

void qwe(void)
{
  // your init code here

  // Abi messages bindings
  AbiBindMsgACCEL_SP(QWE_ID, &qwe_ev, qwe_cb);
}

void qwe(void)
{
  // your periodic code here.
  // freq = 50.0 Hz
}

void qwe(void)
{
  // your periodic start code here.
}

void qwe(void)
{
  // your periodic stop code here.
}

void qwe(void)
{
  // your event code here
}


