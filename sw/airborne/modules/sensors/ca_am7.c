*
 * Copyright (C) 2022 OpenUAS
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
 * @file "modules/ca_am7.c"
 * @author OpenUAS
 */

#include "modules/sensors/ca_am7.h"
#include "pprzlink/pprz_transport.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include <time.h>
#include <sys/time.h>
#include "modules/core/abi.h"
#include "modules/airborne_docking/airborne_docking_overactuated.h"

static abi_event AM7_out;
uint8_t sending_msg_id;
struct am7_data_in myam7_data_in;
struct am7_data_out myam7_data_out;
float extra_data_in[255], extra_data_out[255];
uint16_t buffer_in_counter = 0;
uint32_t missed_packets = 0;
uint16_t ca7_message_frequency_RX = 0;
uint32_t received_packets = 0;
float last_ts = 0;
static uint8_t am7_msg_buf_in[sizeof(struct am7_data_in)*2]  __attribute__((aligned));   ///< The message buffer for the device chosen to be 2* message_size total


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void am7_downlink(struct transport_tx *trans, struct link_device *dev)
{
    int16_t pi_translation_x = myam7_data_in.pi_translation_x;
    int16_t pi_translation_y = myam7_data_in.pi_translation_y;
    int16_t pi_translation_z = myam7_data_in.pi_translation_z;
    int16_t pi_rotation_x = myam7_data_in.pi_rotation_x;
    int16_t pi_rotation_y = myam7_data_in.pi_rotation_y;
    int16_t pi_rotation_z = myam7_data_in.pi_rotation_z;
    float rolling_msg_in_telemetry = myam7_data_in.rolling_msg_in;
    uint8_t rolling_msg_in_id_telemetry = myam7_data_in.rolling_msg_in_id;

	   pprz_msg_send_AM7_IN(trans, dev, AC_ID, &pi_translation_x, &pi_translation_y, &pi_translation_z,
		 	  &pi_rotation_x, &pi_rotation_y, &pi_rotation_z, &missed_packets, &ca7_message_frequency_RX,
              &rolling_msg_in_telemetry, &rolling_msg_in_id_telemetry);
}
static void am7_uplink(struct transport_tx *trans, struct link_device *dev)
{

    int16_t for_now_nothing = myam7_data_out.motor_1_state_int;
    float rolling_msg_out_telemetry = myam7_data_out.rolling_msg_out;
    uint8_t rolling_msg_out_id_telemetry = myam7_data_out.rolling_msg_out_id;

	   pprz_msg_send_AM7_OUT(trans, dev, AC_ID, &for_now_nothing, &rolling_msg_out_telemetry, &rolling_msg_out_id_telemetry);

}
#endif

static void data_AM7_out(uint8_t sender_id __attribute__((unused)), struct am7_data_out * myam7_data_out_ptr, float * extra_data_out_ptr){

    memcpy(&myam7_data_out,myam7_data_out_ptr,sizeof(struct am7_data_out));
    memcpy(&extra_data_out,extra_data_out_ptr, sizeof(extra_data_out) );

    //Increase the counter to track the sending messages:
    myam7_data_out.rolling_msg_out = extra_data_out[sending_msg_id];
    myam7_data_out.rolling_msg_out_id = sending_msg_id;
    sending_msg_id++;
    if(sending_msg_id == 255){
        sending_msg_id = 0;
    }

    //Send the message over serial to the Raspberry pi:
    uint8_t *buf_send = (uint8_t *)&myam7_data_out;
    //Calculating the checksum
    uint8_t checksum_out_local = 0;
    for(uint16_t i = 0; i < sizeof(struct am7_data_out) - 1; i++){
        checksum_out_local += buf_send[i];
    }
    myam7_data_out.checksum_out = checksum_out_local;
    //Send bytes
    uart_put_byte(&(AM7_PORT), 0, START_BYTE);
    for(uint8_t i = 0; i < sizeof(struct am7_data_out) ; i++){
        uart_put_byte(&(AM7_PORT), 0, buf_send[i]);
    }
}

void am7_init() 
{
    buffer_in_counter = 0;
    sending_msg_id = 0;

    //Init abi bind msg:
    AbiBindMsgAM7_DATA_OUT(ABI_BROADCAST, &AM7_out, data_AM7_out);

 #if PERIODIC_TELEMETRY
   register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AM7_IN, am7_downlink);
   register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AM7_OUT, am7_uplink);
 #endif
}

/* Parse the InterMCU message */
void am7_parse_msg_in(void)
{
    memcpy(&myam7_data_in, &am7_msg_buf_in[1], sizeof(struct am7_data_in));
    //Assign the rolling message:
    extra_data_in[myam7_data_in.rolling_msg_in_id] = myam7_data_in.rolling_msg_in;
    //Send msg through ABI:
    AbiSendMsgAM7_DATA_IN(ABI_AM7_DATA_IN_ID, &myam7_data_in, &extra_data_in[0]);
}

/* We need to wait for incoming messages */
void am7_event()
{
    if(fabs(get_sys_time_float() - last_ts) > 5){
        received_packets = 0;
        last_ts = get_sys_time_float();
    }
    while(uart_char_available(&(AM7_PORT)) > 0) {
        uint8_t am7_byte_in;
        am7_byte_in = uart_getch(&(AM7_PORT));
        if ((am7_byte_in == START_BYTE) || (buffer_in_counter > 0)) {
            am7_msg_buf_in[buffer_in_counter] = am7_byte_in;
            buffer_in_counter++;
        }
        if (buffer_in_counter > sizeof(struct am7_data_in) ) {
            buffer_in_counter = 0;
            uint8_t checksum_in_local = 0;
            for(uint16_t i = 1; i < sizeof(struct am7_data_in) ; i++){
                checksum_in_local += am7_msg_buf_in[i];
            }
            if(checksum_in_local == am7_msg_buf_in[sizeof(struct am7_data_in)]){
                am7_parse_msg_in();
                received_packets++;
                            }
            else {
                missed_packets++;
            }
        }
    }
    ca7_message_frequency_RX = (uint16_t) received_packets/(get_sys_time_float() - last_ts);
}