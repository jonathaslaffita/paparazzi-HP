/*
 * Copyright (C) Paparazzi Team
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
 * @file "modules/sensors/ca_am7.h"
 * @author Alessandro Mancinelli
 * Converts telemtry data from a CA device AM7 type to the autopilot
 */

#ifndef AM7_H
#define AM7_H

#define START_BYTE 0x9B  //1st start block identifier byte


#include "std.h"
#include <stdbool.h>
#include <stdlib.h>
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"


struct __attribute__((__packed__)) am7_data_in {
    //Motor command
	int16_t pi_translation_x;
	int16_t pi_translation_y;
  int16_t pi_translation_z;
	int16_t pi_rotation_x;
	int16_t pi_rotation_y;
	int16_t pi_rotation_z;
  float rolling_msg_in;
  uint8_t rolling_msg_in_id;
  uint8_t checksum_in;
	};

struct __attribute__((__packed__)) am7_data_out {
    //Actuator state
    int16_t for_now_nothing;
    float rolling_msg_out;
    uint8_t rolling_msg_out_id;
    uint8_t checksum_out;
    
};

extern void am7_init(void);
extern void am7_event(void);

#endif