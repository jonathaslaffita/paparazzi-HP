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

/** @file "modules/airborne_docking/airborne_docking_overactuated.h"
 * @author Jonathas Laffita <jonathaslvdh@gmail.com>
 * Airborne docking module for relative setpoint formulation and acceleration command
 */

#ifndef AIRBORNE_DOCKING_OVERACTUATED_H
#define AIRBORNE_DOCKING_OVERACTUATED_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

extern void airborne_docking_init(void);
extern void airborne_docking_periodic(void);
extern void gps_other_ac_datalink_parse_REMOTE_GPS_SMALL(uint8_t *buf);


// settings for airborne guidance
// extern float guidance_indi_pos_gain;
// extern float guidance_indi_speed_gain;
// extern float guidance_indi_max_bank;

#endif /* GUIDANCE_INDI_H */

