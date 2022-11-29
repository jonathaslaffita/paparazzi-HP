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
#include "state.h"
#include "modules/datalink/datalink.h"
#include "modules/datalink/downlink.h"
#include "modules/radio_control/radio_control.h"
#include "modules/gps/gps_datalink.h"


//include "RELATIVE_POSE_CALCULATION.H"   ////NEEDED

/// MY DEFINES
#ifndef GUIDANCE_INDI_ACCEL_SP_ID
#define GUIDANCE_INDI_ACCEL_SP_ID ABI_BROADCAST      
#endif

#ifndef RELATIVE_POSE_ID
#define RELATIVE_POSE_ID ABI_BROADCAST
#endif

#define AIRBORNE_DOCKING_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[Airborne_Docking->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if AIRBORNE_DOCKING_VERBOSE                               //NEEDED?
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

struct FloatVect3 a_wanted = {0.0,0.0,0.0};
struct FloatVect3 relative_position_wanted = {-2.0,0.f,1};                       //m in the leader BODY FRAME




enum docking_state_t {                         //WHAT CAN I USE THIS FOR?
  LOOKING_MANUAL,
  TARGET_FOUND_PRECONTACT_SETPOINT,
  TERMINAL_GUIDANCE,
  JUMP
};

// define settings
int32_t flag;
int32_t LEADER_AC_ID = 6;
struct FloatVect3 relative_position_NED;
float relative_heading;                                        //rad? deg?
float relative_position_error_threshold;                       // RELATIVE POSITION NED
struct FloatVect3 relative_position_P_GAIN = {2.0,2.0,2.0};       // Relative position GAIN
struct FloatVect3 relative_position_D_GAIN = {4.0,4.0,4.0};
float relative_velocity_speed_gain = 0.05f;       // relative velocity GAIN
float relative_accel_gain;                        //raltive acceleration GAIN
float heading_setpoint;             // heading setpoint 
float transversal_error;
float min_error;
float max_error;
struct FloatMat33 ROT_inv;
struct FloatVect3 docked_setpoint = {0.1 , 0.0, -0.1};//{-distance_of_docked_setpoint * cos(angle_of_aproach), 0 , -distance_of_docked_setpoint * sin(angle_of_aproach)};
struct FloatVect3 pre_docked_setpoint = {-1.5, 0.-1.0};//{-distance_of_docked_setpoint * cos(angle_of_aproach), 0 , -distance_of_docked_setpoint * sin(angle_of_aproach)};
struct FloatVect3 error_to_setpoint_NED;
struct FloatVect3 prev_error_to_setpoint_NED;
struct FloatVect3 derrivative_error_to_setpoint_NED;
float advance = -0.5;
struct GpsState gps_leader_AC_datalink;
struct Int32Vect3 AC_XYZ;
struct Int32Vect3 AC_XYZ_SPEED;
struct Int32Vect3 LEADER_AC_XYZ;
struct Int32Vect3 LEADER_AC_XYZ_SPEED;
struct FloatVect3 relative_position_xyz;

/////// FOR GPS INITIALIZATION
// struct LtpDef_i ltp_def;


                                                  //what else

//initializes some global variables
int32_t IRLOCK = 0;
int32_t GOOD_PRECONTACT;
int32_t IRLOCK_confidence = -8;
enum docking_state_t docking_state = LOOKING_MANUAL;   // current state in state machine
// static abi_event qwe_ev;
int32_t ALIVE = 0;
// DECLARATIONS OF FORMULAE
void processrelativepose(void);
void get_terminal_relative_sp(void);
void calc_accel_sp(void);
// void parse_gps_datalink_small_22(int16_t heading, uint32_t pos_xyz, uint32_t speed_xyz, uint32_t tow);
// extern void gps_other_ac_datalink_parse_REMOTE_GPS_SMALL(uint8_t *buf);


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_airborne_docking(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AIRBORNE_DOCKING(trans, dev, AC_ID,
                              &ALIVE,
                              &LEADER_AC_XYZ.x,
                              &LEADER_AC_XYZ.y,
                              &LEADER_AC_XYZ.z,
                              &AC_XYZ.x,
                              &AC_XYZ.y,
                              &AC_XYZ.z,
                              &relative_position_NED.x,
                              &relative_position_NED.y,
                              &relative_position_NED.z);
}
#endif


void airborne_docking_init(void)
{
  // your init code here
  // gps_datalink_init();
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRBORNE_DOCKING, send_airborne_docking);
  #endif

  float heading  = stateGetNedToBodyEulers_f()->psi;    //getting heading
  // AbiBindMsgRELATIVE_POSE(ORANblahhblhaISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);  ///// receive  POSE?? in body frame
  // Abi messages bindings
  // AbiBindMsgACCEL_SP(QWE_ID, &qwe_ev, qwe_cb);       /// not needed!!?
  

  
}

 
void airborne_docking_periodic(void)
{

  processrelativepose();
  

  VERBOSE_PRINT("state: %d \n", docking_state);

  // update our confidence if confidence that we are indeed seing follower if several PNPs in a row say that is the case   (NEEDED)
  if(IRLOCK < 1){
    IRLOCK_confidence -= 1;
  } else {
    IRLOCK_confidence ++; 
  }
    // bound obstacle_free_confidence
  Bound(IRLOCK_confidence, -10, 10);


    // if no stability reached, keep trying to get to precontact.
  if(transversal_error < relative_position_error_threshold){     //FIND THIS (IF ERROR INSIDE PRECONTACT BOX SMALLER THAN A CERTAIN AMOUNT, INCREASE PRECONTACT_confidence)
    advance -= 0.01;
  } else {
   advance += 0.01; 
  }
  Bound(advance, -1,1);
    // bound obstacle_free_confidence
  Bound(advance, -1, 1);

  //float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);  interesting concept

  switch (docking_state){

    case LOOKING_MANUAL:   //STATE MODULE ON, BUT STILL PILOTING AIRCRAFT MANUALLY (HOW DOES THIS PLAY INTO THE HANDS OF GUIDANCE_INDI (THOUGH ABI_SETPOINT TOO?))
      
      if (IRLOCK_confidence >= 0){
        docking_state = TARGET_FOUND_PRECONTACT_SETPOINT;
      }

      //for direct rc control horizontal, rotate from body axes to NED (MAYBE IN THE FUTRE REMOVE THE NEED FOR PSI??)
      float psi = stateGetNedToBodyEulers_f()->psi;
      float rc_x = -(radio_control.values[RADIO_PITCH] / 9600.0) * 6.0;
      float rc_y = (radio_control.values[RADIO_ROLL] / 9600.0) * 6.0;
      a_wanted.x = cosf(psi) * rc_x - sinf(psi) * rc_y;
      a_wanted.y = sinf(psi) * rc_x + cosf(psi) * rc_y;
      a_wanted.z = -(radio_control.values[RADIO_THROTTLE] - 4500) * 6.0 / 9600.0;
      break;  //do nothing? keep piloting

           
    case TARGET_FOUND_PRECONTACT_SETPOINT:   //setpoint is now precontact position

      relative_position_wanted = pre_docked_setpoint;

      if (IRLOCK_confidence <= 0){
        docking_state = LOOKING_MANUAL;
        break;
      } else if(advance >= 0){
        docking_state = TERMINAL_GUIDANCE;
        break;
      }else{
            get_terminal_relative_sp(); 
            calc_accel_sp();
            AbiSendMsgACCEL_SP(ABI_BROADCAST, flag, &a_wanted);
      }
 
      break;

    case TERMINAL_GUIDANCE:
      if (IRLOCK_confidence <= 0){
        docking_state = LOOKING_MANUAL;
        break;
      } else if(advance <= 0){
        docking_state = TARGET_FOUND_PRECONTACT_SETPOINT;
        break;
      }else{
      get_terminal_relative_sp(); 
      calc_accel_sp();
      AbiSendMsgACCEL_SP(ABI_BROADCAST, flag, &a_wanted);

      }
      break;
      
    case JUMP:

      break;
    
  }
}
  
/*
* Function that calculates relative pose + RELATIVE STATES, finds the errors and calculates derrivatives including relative accelerations
*/
 // from VISION WE GET RELATIVE POSE + OPTICAL FLOW. 
 // this will get this information from the camera frame to the NED navigation frame (maybe I can ignor ROTATIONS FOR NOW AND FOCUS ON DISTANCE + RELATIVE HEADING)
void processrelativepose()
{
struct FloatVect3 relative_position_bframe;
struct FloatVect3 leader_relative_position_wanted_NED;
float psi = stateGetNedToBodyEulers_f()->psi;
float leader_psi_diff = gps_leader_AC_datalink.course - psi;                               //add + optitrack
float psi_target = psi + leader_psi_diff;
struct FloatRMat *matforrelativeNED = stateGetNedToBodyRMat_f();
MAT33_TRANS(ROT_inv, *matforrelativeNED);
AC_XYZ.x = stateGetPositionNed_i()->x;
AC_XYZ.y = stateGetPositionNed_i()->y;
AC_XYZ.z = stateGetPositionNed_i()->z;


VECT3_DIFF(relative_position_xyz, LEADER_AC_XYZ, AC_XYZ);
VECT3_SMUL(relative_position_xyz,relative_position_xyz,10);
relative_position_NED = relative_position_xyz;                           //MAYBE SHIFT BY HEADING
// MAT33_VECT3_MUL(relative_position_NED, ROT_inv, relative_position_bframe);   TO USE WITH BFRAME
leader_relative_position_wanted_NED.x = relative_position_wanted.x * cosf(psi) - sinf(psi) * relative_position_wanted.y;
leader_relative_position_wanted_NED.y = relative_position_wanted.y * sinf(psi) + cosf(psi) * relative_position_wanted.y;
leader_relative_position_wanted_NED.z = relative_position_wanted.z;
prev_error_to_setpoint_NED = error_to_setpoint_NED;

VECT3_DIFF(error_to_setpoint_NED, relative_position_NED, leader_relative_position_wanted_NED);
VECT3_DIFF(derrivative_error_to_setpoint_NED, error_to_setpoint_NED, prev_error_to_setpoint_NED);
VECT3_SMUL(derrivative_error_to_setpoint_NED, derrivative_error_to_setpoint_NED, PERIODIC_FREQUENCY)

transversal_error = sqrt((pow(error_to_setpoint_NED.x,2) + pow(error_to_setpoint_NED.y,2)));
}

/*
* Function that calculates position setpoint in the terminal guidance (body_frame)
*/
void get_terminal_relative_sp(void){
// this formula uses calculated error to setpoint from proceesrelative pose and changes the required setpoint based on it 
// float distance_to_AC_wanted = docked_setpoint - ((transversal_error-min_error)*(pre_dock_distance_longitudinal + docked_setpoint)^4/(max_error))^0.25;
// float height_to_AC_wanted = (transversal_error - min_error)/(max_error-min_error)*PREDOCK_Z;
if (advance <= 0)
{
  relative_position_wanted = pre_docked_setpoint;
}
else
{
  VECT3_DIFF(relative_position_wanted, docked_setpoint, pre_docked_setpoint);
  // MAT33_MULT_SCALAR(relative_position_wanted,(transversal_error - min_error)/(max_error-min_error));
  VECT3_SMUL(relative_position_wanted,relative_position_wanted, (1-advance));
}}

/*
* Function that calculates setpoint acceleration from terminal setpoint
*/
void calc_accel_sp(void){
// FOR NOW JUST A GAIN KXYZ TO THE ERROR

a_wanted.x = error_to_setpoint_NED.x * relative_position_P_GAIN.x + derrivative_error_to_setpoint_NED.x * relative_position_D_GAIN.x;
a_wanted.y = error_to_setpoint_NED.y * relative_position_P_GAIN.y + derrivative_error_to_setpoint_NED.y * relative_position_D_GAIN.y;
a_wanted.z = error_to_setpoint_NED.z * relative_position_P_GAIN.z + derrivative_error_to_setpoint_NED.z * relative_position_D_GAIN.z;

}


// Parse the REMOTE_GPS_SMALL datalink packet
static void parse_gps_datalink_small_2(int16_t heading, uint32_t pos_xyz, uint32_t speed_xyz, uint32_t tow)
{
  struct EnuCoor_i enu_pos, enu_speed;

  // Position in ENU coordinates
  enu_pos.x = (int32_t)((pos_xyz >> 21) & 0x7FF); // bits 31-21 x position in cm
  if (enu_pos.x & 0x400) {
    enu_pos.x |= 0xFFFFF800;  // sign extend for twos complements
  }
  enu_pos.y = (int32_t)((pos_xyz >> 10) & 0x7FF); // bits 20-10 y position in cm
  if (enu_pos.y & 0x400) {
    enu_pos.y |= 0xFFFFF800;  // sign extend for twos complements
  }
  enu_pos.z = (int32_t)(pos_xyz & 0x3FF); // bits 9-0 z position in cm

  LEADER_AC_XYZ.x = enu_pos.x;
  LEADER_AC_XYZ.y = enu_pos.y;
  LEADER_AC_XYZ.z = enu_pos.z;


  // Convert the ENU coordinates to ECEF
  ecef_of_enu_point_i(&gps_leader_AC_datalink.ecef_pos, &ltp_def, &enu_pos);
  SetBit(gps_leader_AC_datalink.valid_fields, GPS_VALID_POS_ECEF_BIT);

  lla_of_ecef_i(&gps_leader_AC_datalink.lla_pos, &gps_leader_AC_datalink.ecef_pos);
  SetBit(gps_leader_AC_datalink.valid_fields, GPS_VALID_POS_LLA_BIT);

  enu_speed.x = (int32_t)((speed_xyz >> 21) & 0x7FF); // bits 31-21 speed x in cm/s
  if (enu_speed.x & 0x400) {
    enu_speed.x |= 0xFFFFF800;  // sign extend for twos complements
  }
  enu_speed.y = (int32_t)((speed_xyz >> 10) & 0x7FF); // bits 20-10 speed y in cm/s
  if (enu_speed.y & 0x400) {
    enu_speed.y |= 0xFFFFF800;  // sign extend for twos complements
  }
  enu_speed.z = (int32_t)((speed_xyz) & 0x3FF); // bits 9-0 speed z in cm/s
  if (enu_speed.z & 0x200) {
    enu_speed.z |= 0xFFFFFC00;  // sign extend for twos complements
  }

  VECT3_NED_OF_ENU(gps_leader_AC_datalink.ned_vel, enu_speed);
  SetBit(gps_leader_AC_datalink.valid_fields, GPS_VALID_VEL_NED_BIT);

  ecef_of_enu_vect_i(&gps_leader_AC_datalink.ecef_vel , &ltp_def , &enu_speed);
  SetBit(gps_leader_AC_datalink.valid_fields, GPS_VALID_VEL_ECEF_BIT);

  gps_leader_AC_datalink.gspeed = (int16_t)FLOAT_VECT2_NORM(enu_speed);
  gps_leader_AC_datalink.speed_3d = (int16_t)FLOAT_VECT3_NORM(enu_speed);

  gps_leader_AC_datalink.hmsl = ltp_def.hmsl + enu_pos.z * 10;
  SetBit(gps_leader_AC_datalink.valid_fields, GPS_VALID_HMSL_BIT);

  gps_leader_AC_datalink.course = ((int32_t)heading) * 1e3;
  SetBit(gps_leader_AC_datalink.valid_fields, GPS_VALID_COURSE_BIT);

  gps_leader_AC_datalink.num_sv = 30;
  gps_leader_AC_datalink.tow = tow;
  gps_leader_AC_datalink.fix = GPS_FIX_3D; // set 3D fix to true

  // set gps msg time
  gps_leader_AC_datalink.last_msg_ticks = sys_time.nb_sec_rem;
  gps_leader_AC_datalink.last_msg_time = sys_time.nb_sec;

  gps_leader_AC_datalink.last_3dfix_ticks = sys_time.nb_sec_rem;
  gps_leader_AC_datalink.last_3dfix_time = sys_time.nb_sec;

  // publish new GPS data
  uint32_t now_ts = get_sys_time_usec();
 }



// void parse_gps_datalink_small_2(int16_t heading, uint32_t pos_xyz, uint32_t speed_xyz, uint32_t tow)
// {
//   struct EnuCoor_i enu_pos, enu_speed;


//   // Position in ENU coordinates
//   LEADER_AC_XYZ.x = (int32_t)((pos_xyz >> 21) & 0x7FF); // bits 31-21 x position in cm
//   if (enu_pos.x & 0x400) {
//     LEADER_AC_XYZ.x |= 0xFFFFF800;  // sign extend for twos complements
//   }
//   LEADER_AC_XYZ.y = (int32_t)((pos_xyz >> 10) & 0x7FF); // bits 20-10 y position in cm
//   if (enu_pos.y & 0x400) {
//     LEADER_AC_XYZ.y |= 0xFFFFF800;  // sign extend for twos complements
//   }
//   LEADER_AC_XYZ.z = (int32_t)(pos_xyz & 0x3FF); // bits 9-0 z position in cm

//   LEADER_AC_XYZ_SPEED.x = (int32_t)((speed_xyz >> 21) & 0x7FF); // bits 31-21 speed x in cm/s
//   if (enu_speed.x & 0x400) {
//     LEADER_AC_XYZ_SPEED.x |= 0xFFFFF800;  // sign extend for twos complements
//   }
//   LEADER_AC_XYZ_SPEED.y = (int32_t)((speed_xyz >> 10) & 0x7FF); // bits 20-10 speed y in cm/s
//   if (enu_speed.y & 0x400) {
//     LEADER_AC_XYZ_SPEED.y |= 0xFFFFF800;  // sign extend for twos complements
//   }
//   LEADER_AC_XYZ_SPEED.z = (int32_t)((speed_xyz) & 0x3FF); // bits 9-0 speed z in cm/s
//   if (enu_speed.z & 0x200) {
//     LEADER_AC_XYZ_SPEED.z |= 0xFFFFFC00;  // sign extend for twos complements
//   }

//   // VECT3_NED_OF_ENU(gps_leader_AC_datalink.ned_vel, enu_speed);
  
//   gps_leader_AC_datalink.course = ((int32_t)heading) * 1e3;
  
// }

// void parse_gps_datalink_small_3(int16_t heading, uint32_t pos_xyz, uint32_t speed_xyz, uint32_t tow)
// {
//   struct EnuCoor_i enu_pos, enu_speed;


//   // Position in ENU coordinates
//   AC_XYZ.x = (int32_t)((pos_xyz >> 21) & 0x7FF); // bits 31-21 x position in cm
//   if (enu_pos.x & 0x400) {
//     AC_XYZ.x |= 0xFFFFF800;  // sign extend for twos complements
//   }
//   AC_XYZ.y = (int32_t)((pos_xyz >> 10) & 0x7FF); // bits 20-10 y position in cm
//   if (enu_pos.y & 0x400) {
//     AC_XYZ.y |= 0xFFFFF800;  // sign extend for twos complements
//   }
//   AC_XYZ.z = (int32_t)(pos_xyz & 0x3FF); // bits 9-0 z position in cm

//   AC_XYZ_SPEED.x = (int32_t)((speed_xyz >> 21) & 0x7FF); // bits 31-21 speed x in cm/s
//   if (enu_speed.x & 0x400) {
//     AC_XYZ_SPEED.x |= 0xFFFFF800;  // sign extend for twos complements
//   }
//   AC_XYZ_SPEED.y = (int32_t)((speed_xyz >> 10) & 0x7FF); // bits 20-10 speed y in cm/s
//   if (enu_speed.y & 0x400) {
//     AC_XYZ_SPEED.y |= 0xFFFFF800;  // sign extend for twos complements
//   }
//   AC_XYZ_SPEED.z = (int32_t)((speed_xyz) & 0x3FF); // bits 9-0 speed z in cm/s
//   if (enu_speed.z & 0x200) {
//     AC_XYZ_SPEED.z |= 0xFFFFFC00;  // sign extend for twos complements
//   }

  // VECT3_NED_OF_ENU(gps_leader_AC_datalink.ned_vel, enu_speed);
  
  // gps_leader_AC_datalink.course = ((int32_t)heading) * 1e3;
  
// }
// THIS IS THE INIT FUNCTION FOR GPS!
// void gps_datalink_init(void)
// {
//   gps_datalink.fix = GPS_FIX_NONE;
//   gps_datalink.pdop = 10;
//   gps_datalink.sacc = 5;
//   gps_datalink.pacc = 1;
//   gps_datalink.cacc = 1;

//   gps_datalink.comp_id = GPS_DATALINK_ID;

//   struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
//   llh_nav0.lat = NAV_LAT0;
//   llh_nav0.lon = NAV_LON0;
//   /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
//   llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

//   ltp_def_from_lla_i(&ltp_def, &llh_nav0);
// }

extern void gps_other_ac_datalink_parse_REMOTE_GPS_SMALL(uint8_t *buf)
{
  // if (DL_REMOTE_GPS_LOCAL_ac_id(buf) != AC_ID) { return; } // not for this aircraft
  ALIVE = 1;
  parse_gps_datalink_small_2(DL_REMOTE_GPS_SMALL_heading(buf),
                            DL_REMOTE_GPS_SMALL_pos_xyz(buf),
                            DL_REMOTE_GPS_SMALL_speed_xyz(buf),
                            DL_REMOTE_GPS_SMALL_tow(buf));
}


// extern void gps_other_ac_datalink_parse_REMOTE_GPS_SMALL(uint8_t *buf)
// {
//   ALIVE = DL_REMOTE_GPS_SMALL_ac_id(buf);
//   if (DL_REMOTE_GPS_SMALL_ac_id(buf) == LEADER_AC_ID){
//   parse_gps_datalink_small_2(DL_REMOTE_GPS_SMALL_heading(buf),
//                             DL_REMOTE_GPS_SMALL_pos_xyz(buf),
//                             DL_REMOTE_GPS_SMALL_speed_xyz(buf),
//                             DL_REMOTE_GPS_SMALL_tow(buf));
//   }else if(DL_REMOTE_GPS_SMALL_ac_id(buf) == AC_ID){
//   parse_gps_datalink_small_3(DL_REMOTE_GPS_SMALL_heading(buf),
//                             DL_REMOTE_GPS_SMALL_pos_xyz(buf),
//                             DL_REMOTE_GPS_SMALL_speed_xyz(buf),
//                             DL_REMOTE_GPS_SMALL_tow(buf));
//   }
//   else { return;} // not for these aircraft