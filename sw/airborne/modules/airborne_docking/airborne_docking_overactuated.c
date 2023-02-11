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
#include "modules/sensors/ca_am7.h"
// #include <sys/time.h>


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
struct FloatVect3 relative_position_wanted = {-2.0,0.0,1.0};                       //m in the leader BODY FRAME
struct FloatVect3 approach_vector = {-2.0,0.0,1.0};



enum docking_state_t {                         //WHAT CAN I USE THIS FOR?
  LOOKING_MANUAL,
  TARGET_FOUND_PRECONTACT_SETPOINT,
  TERMINAL_GUIDANCE,
  DROPOUT_USED_GPS,
  JUMP
};

// define settings
int16_t leader_psi_diff=0;
int32_t flag = 0; //  2D for now
int32_t LEADER_AC_ID = 6;
float relative_heading;                                        //rad? deg?
float relative_position_error_threshold = 0.0000001;                       // RELATIVE POSITION NED
extern float relative_position_P_GAIN_X = 0.5;
extern float relative_position_P_GAIN_Y = 0.5;
extern float relative_position_P_GAIN_Z = 0.5;
struct FloatVect3 relative_position_P_GAINS = {0.5,0.5,0.5};       // Relative position GAIN
extern float relative_position_D_GAIN_X = 1.8;
extern float relative_position_D_GAIN_Y = 1.8;
extern float relative_position_D_GAIN_Z = 1.8;
struct FloatVect3 relative_position_D_GAINS = {1.8,1.8,1.8};
float extern velocity_speed_gain = 2.0;       // relative velocity GAIN
float relative_accel_gain;                        //raltive acceleration GAIN
float heading_setpoint;             // heading setpoint 
float transversal_error = 2000;
// float min_error;
// float max_error;
struct FloatMat33 ROT_inv;
struct FloatVect3 docked_setpoint = {-0.5 , 0.0, 0.0};//{-distance_of_docked_setpoint * cos(angle_of_aproach), 0 , -distance_of_docked_setpoint * sin(angle_of_aproach)};
extern float pre_docked_setpoint_X= -1.5;
extern float pre_docked_setpoint_Y = 0.0;
extern float pre_docked_setpoint_Z = 0.0;
struct FloatVect3 pre_docked_setpoint = {-1.5, 0.0,1.0};//{-distance_of_docked_setpoint * cos(angle_of_aproach), 0 , -distance_of_docked_setpoint * sin(angle_of_aproach)};
struct FloatVect3 error_to_setpoint_NED;
struct FloatVect3 prev_error_to_setpoint_NED;
struct FloatVect3 derrivative_error_to_setpoint_NED;
float advance = -0.5;
struct GpsState gps_leader_AC_datalink;
struct FloatVect3 AC_NED;
struct FloatVect3 AC_NED_SPEED;
struct FloatVect3 LEADER_AC_NED;
struct FloatVect3 LEADER_AC_NED_SPEED;
struct FloatVect3 relative_position_NED;
struct FloatVect3 relative_speed_NED;
float psi_target;
struct FloatVect3 relative_position_bframe;
struct FloatVect3 leader_relative_position_wanted_NED;
struct FloatVect3 velocity_wanted;
// struct FloatMat33 pitobody_RMAT;
// struct FloatVect3 error_to_velocity_setpoint;
// struct FloatVect3 derrivative_error_to_velocity_setpoint;
// float running_frequency = 5.0;
struct Int16Vect3 pi_relative_distance;
struct Int16Vect3 pi_relative_speed;
struct Int16Vect3 pi_relative_pose;
struct FloatVect3 pi_relative_distance_body_frame;

/////// FOR GPS INITIALIZATION
// struct LtpDef_i ltp_def;


float extra_data_in_local[255];
struct am7_data_in pi_vision;
static abi_event AM7_in;

                                                  //what else

//initializes some global variables
int32_t IRLOCK = 1;         //for now it is forced at +1
int32_t IRLOCK_confidence = -8;
docking_state = DROPOUT_USED_GPS;   // current state in state machine  enum docking_state_t  removed from the front
// static abi_event qwe_ev;
int32_t ALIVE = 0;
// DECLARATIONS OF FORMULAE
// void processrelativepose(void);
void processrelativeposegps(void);
void processrelativeposevision(void);
void get_terminal_relative_sp(void);
void calc_accel_sp(void);


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_airborne_docking(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AIRBORNE_DOCKING(trans, dev, AC_ID,
                              &advance,
                              &LEADER_AC_NED.x,
                              &LEADER_AC_NED.y,
                              &LEADER_AC_NED.z,
                              &AC_NED.x,
                              &AC_NED.y,
                              &AC_NED.z,
                              &relative_position_NED.x,
                              &relative_position_NED.y,
                              &relative_position_NED.z,
                              &relative_position_wanted.x,
                              &relative_position_wanted.y,
                              &relative_position_wanted.z,
                              &leader_relative_position_wanted_NED.x,
                              &leader_relative_position_wanted_NED.y,
                              &leader_relative_position_wanted_NED.z,
                              &error_to_setpoint_NED.x,
                              &error_to_setpoint_NED.y,
                              &error_to_setpoint_NED.z,
                              &derrivative_error_to_setpoint_NED.x,
                              &derrivative_error_to_setpoint_NED.y,
                              &derrivative_error_to_setpoint_NED.z,
                              &velocity_wanted.x,
                              &velocity_wanted.y,
                              &velocity_wanted.z,
                              &pi_relative_distance.x,
                              &pi_relative_distance.y,
                              &pi_relative_distance.z
                              // &pi_relative_distance_body_frame.x,
                              // &pi_relative_distance_body_frame.y,
                              // &pi_relative_distance_body_frame.z
                              );
}
#endif


static void data_AM7_abi_in(uint8_t sender_id __attribute__((unused)), struct am7_data_in * myam7_data_in_ptr, float * extra_data_in_ptr){
    memcpy(&pi_vision,myam7_data_in_ptr,sizeof(struct am7_data_in));
    // memcpy(&extra_data_in_local,extra_data_in_ptr,255 * sizeof(float));
    airborne_docking_periodic();
    pi_relative_distance.x = pi_vision.pi_translation_x;
    pi_relative_distance.y = pi_vision.pi_translation_y;
    pi_relative_distance.z = pi_vision.pi_translation_z;
    pi_relative_speed.x = pi_vision.pi_translation_speed_x;
    pi_relative_speed.y = pi_vision.pi_translation_speed_y;
    pi_relative_speed.z = pi_vision.pi_translation_speed_z;
    pi_relative_pose.x = pi_vision.pi_rotation_x;
    pi_relative_pose.y = pi_vision.pi_rotation_y;
    pi_relative_pose.z = pi_vision.pi_rotation_z;
    IRLOCK = pi_vision.pivision_flag;
}

void airborne_docking_init(void)
{
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRBORNE_DOCKING, send_airborne_docking);
  #endif
  AbiBindMsgAM7_DATA_IN(ABI_BROADCAST, &AM7_in, data_AM7_abi_in);
  // AbiBindMsgRELATIVE_POSE(ORANblahhblhaISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);  ///// receive  POSE?? in body frame
  
  // RMAT_ELMT(pitobody_RMAT, 0, 0) = cosf(0.36332);
  // RMAT_ELMT(pitobody_RMAT, 1, 0) = 0;
  // RMAT_ELMT(pitobody_RMAT, 2, 0) = sinf(0.436332);
  // RMAT_ELMT(pitobody_RMAT, 0, 1) = 0;
  // RMAT_ELMT(pitobody_RMAT, 1, 1) = 1.0;
  // RMAT_ELMT(pitobody_RMAT, 2, 1) = 0;
  // RMAT_ELMT(pitobody_RMAT, 0, 2) = -sinf(0.436332);
  // RMAT_ELMT(pitobody_RMAT, 1, 2) = 0;
  // RMAT_ELMT(pitobody_RMAT, 2, 2) = cosf(0.436332);

  
}

/**
 * @param heading_sp the desired heading [rad]
 *
 * main airborne docking function
 */
void airborne_docking_periodic(void)
{

  // processrelativepose();  // Maybe here I should run the filter and bypass this timing bullcrap  
  

  VERBOSE_PRINT("state: %d \n", docking_state); //THIS DOES NOT WORK WHEN FLYING REAL AC?

  // update our confidence if confidence that we are indeed seing follower if several PNPs in a row say that is the case   (NEEDED??)
  if(IRLOCK < 1){
    IRLOCK_confidence -= 1;
  } else {
    IRLOCK_confidence ++; 
  }
    // bound obstacle_free_confidence
  Bound(IRLOCK_confidence, -10, 50);


    // if no stability reached, keep trying to get to precontact.
  if(transversal_error < relative_position_error_threshold){     //FIND THIS (IF ERROR INSIDE PRECONTACT BOX SMALLER THAN A CERTAIN AMOUNT, INCREASE PRECONTACT_confidence)
    advance += 0.0001;
  } else {
   advance -= 0.0001;                      //advance number from -1 to 1. at 1 fully docked.
  }

  Bound(advance, -1,1);
    // bound advance
  Bound(advance, -1, 1);

  //float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);  interesting concept

  switch (docking_state){

    case LOOKING_MANUAL:   //STATE MODULE ON, BUT STILL PILOTING AIRCRAFT MANUALLY (HOW DOES THIS PLAY INTO THE HANDS OF GUIDANCE_INDI (THOUGH ABI_SETPOINT TOO?))
      
      if (IRLOCK_confidence >= 0){
        docking_state = TARGET_FOUND_PRECONTACT_SETPOINT;  //CHNAGE TO IF SWITCH ACTIVATED
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
        docking_state = DROPOUT_USED_GPS;
        advance = -1;
        break;
      } else if(advance >= 0){
        docking_state = TERMINAL_GUIDANCE;
        break;
      }else{
            processrelativeposevision();
            get_terminal_relative_sp(); 
            calc_accel_sp();
            AbiSendMsgACCEL_SP(ABI_BROADCAST, flag, &a_wanted);
      }
 
      break;

    case TERMINAL_GUIDANCE:
      if (IRLOCK_confidence <= 0){
        docking_state = DROPOUT_USED_GPS;
        advance = -1;
        break;
      } else if(advance <= 0){
        docking_state = TARGET_FOUND_PRECONTACT_SETPOINT;
        break;
      }else{
        processrelativeposevision();
        get_terminal_relative_sp(); 
        calc_accel_sp();
        AbiSendMsgACCEL_SP(ABI_BROADCAST, flag, &a_wanted);
        }
        break;
      
    case DROPOUT_USED_GPS:
      if (IRLOCK_confidence >= 0){
        docking_state = TARGET_FOUND_PRECONTACT_SETPOINT;
      }
      else{
        processrelativeposegps();
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
void processrelativeposegps()
  {
  float psi = stateGetNedToBodyEulers_f()->psi;
  leader_psi_diff = gps_leader_AC_datalink.course - psi;                               //add + optitrack
  psi_target = psi + leader_psi_diff;
  psi_target = 0;      // not dealing with heading right now.

  // struct FloatRMat *matforrelativeNED = stateGetNedToBodyRMat_f();
  // MAT33_TRANS(ROT_inv, *matforrelativeNED);

  /// @brief  PORTION FOR OPTITRACK
  AC_NED.x = stateGetPositionNed_f()->x *1000;  //in meters
  AC_NED.y = stateGetPositionNed_f()->y * 1000;
  AC_NED.z = -stateGetPositionNed_f()->z * 1000;
  AC_NED_SPEED.x = stateGetSpeedNed_f() ->x * 1000;
  AC_NED_SPEED.y = stateGetSpeedNed_f() ->y * 1000;
  AC_NED_SPEED.z = stateGetSpeedNed_f() ->z * 1000;

  // // Set the default tracking system position and angle
  //   struct LlaCoor_d tracking_lla;
  //   struct LtpDef_d tracking_ltp; 
  //   tracking_lla.lat = RadOfDeg(51.9906340);
  //   tracking_lla.lon = RadOfDeg(4.3767889);
  //   tracking_lla.alt = 45.103;
  //   tracking_offset_angle = 33.0 / 57.6;
  //   ltp_def_from_lla_d(&tracking_ltp, &tracking_lla);



  VECT3_DIFF(relative_position_NED, AC_NED, LEADER_AC_NED);
                          //MAYBE SHIFT BY HEADING
  // MAT33_VECT3_MUL(relative_position_NED, ROT_inv, relative_position_bframe);   TO USE WITH BFRAME
  leader_relative_position_wanted_NED.x = relative_position_wanted.x * cosf(psi_target) - sinf(psi_target) * relative_position_wanted.y;
  leader_relative_position_wanted_NED.y = relative_position_wanted.y * cosf(psi_target) + sinf(psi_target) * relative_position_wanted.y;
  leader_relative_position_wanted_NED.z = relative_position_wanted.z;
  prev_error_to_setpoint_NED = error_to_setpoint_NED;

  VECT3_DIFF(error_to_setpoint_NED, leader_relative_position_wanted_NED, relative_position_NED);
  VECT3_DIFF(relative_speed_NED, AC_NED_SPEED, LEADER_AC_NED_SPEED);
  // VECT3_DIFF(derrivative_error_to_setpoint_NED, error_to_setpoint_NED, prev_error_to_setpoint_NED);

  // VECT3_SMUL(derrivative_error_to_setpoint_NED, derrivative_error_to_setpoint_NED, running_frequency)  //unfiltered diferntiated error to setpoint

  transversal_error = sqrt((pow(error_to_setpoint_NED.x,2) + pow(error_to_setpoint_NED.y,2)));      //what about Z?
  }
 
 
void processrelativeposevision()
{
  float psi = stateGetNedToBodyEulers_f()->psi;
 
  struct FloatRMat *matforrelativeNED = stateGetNedToBodyRMat_f();
  MAT33_TRANS(ROT_inv, *matforrelativeNED);

  leader_psi_diff = pi_relative_pose.z;                               //check if true .z but for psi, check what kind should be used for angles
  psi_target = psi + leader_psi_diff;                                       
  psi_target = 0;                                       // not dealing with heading right now.

  //MAYBE SHIFT BY HEADING
  // MAT33_VECT3_MUL(pi_relative_distance_body_frame, pitobody_RMAT, pi_relative_distance); 

  MAT33_VECT3_MUL(relative_position_NED, ROT_inv, pi_relative_distance);   //TO USE WITH BFRAME
  MAT33_VECT3_MUL(relative_speed_NED, ROT_inv, pi_relative_speed); 

  leader_relative_position_wanted_NED.x = relative_position_wanted.x * cosf(psi_target) - sinf(psi_target) * relative_position_wanted.y;
  leader_relative_position_wanted_NED.y = relative_position_wanted.y * cosf(psi_target) + sinf(psi_target) * relative_position_wanted.y;
  leader_relative_position_wanted_NED.z = relative_position_wanted.z;

  prev_error_to_setpoint_NED = error_to_setpoint_NED;

  VECT3_DIFF(error_to_setpoint_NED, leader_relative_position_wanted_NED, relative_position_NED);
  // VECT3_DIFF(derrivative_error_to_setpoint_NED, error_to_setpoint_NED, prev_error_to_setpoint_NED);
  // VECT3_SMUL(derrivative_error_to_setpoint_NED, derrivative_error_to_setpoint_NED, running_frequency)  //unfiltered diferntiated error to setpoint

  transversal_error = sqrt((pow(error_to_setpoint_NED.x,2) + pow(error_to_setpoint_NED.y,2)));      //what about Z?
  }


  /*
  * Function that calculates position setpoint in the terminal guidance (body_frame)
  */
void get_terminal_relative_sp(void){
// this formula uses calculated error to setpoint from proceesrelative pose and changes the required setpoint based on it 
// float distance_to_AC_wanted = docked_setpoint - ((transversal_error-min_error)*(pre_dock_distance_longitudinal + docked_setpoint)^4/(max_error))^0.25;
// float height_to_AC_wanted = (transversal_error - min_error)/(max_error-min_error)*PREDOCK_Z;
pre_docked_setpoint.x = pre_docked_setpoint_X;
pre_docked_setpoint.y = pre_docked_setpoint_Y;
pre_docked_setpoint.z = pre_docked_setpoint_Z;

if (advance <= 0)
{
  relative_position_wanted = pre_docked_setpoint;
}
else
{
  VECT3_DIFF(approach_vector, docked_setpoint, pre_docked_setpoint);
  // MAT33_MULT_SCALAR(relative_position_wanted,(transversal_error - min_error)/(max_error-min_error));
  VECT3_SMUL(approach_vector, approach_vector, advance);
  VECT3_SUM(relative_position_wanted, pre_docked_setpoint, approach_vector);
}}

/*
* Function that calculates setpoint acceleration from terminal setpoint
*/
void calc_accel_sp(void){
// FOR NOW JUST A GAIN KXYZ TO THE ERROR

relative_position_P_GAINS.x = relative_position_P_GAIN_X;
relative_position_P_GAINS.y = relative_position_P_GAIN_Y;
relative_position_P_GAINS.z = relative_position_P_GAIN_Z;

relative_position_D_GAINS.x = relative_position_D_GAIN_X;
relative_position_D_GAINS.y = relative_position_D_GAIN_Y;
relative_position_D_GAINS.z = relative_position_D_GAIN_Z;

VECT3_SMUL(velocity_wanted, error_to_setpoint_NED, velocity_speed_gain);

static struct FloatVect3 previous_error_to_velocity_setpoint;
// previous_error_to_velocity_setpoint =  error_to_velocity_setpoint;

// VECT3_DIFF(error_to_velocity_setpoint, velocity_wanted, *stateGetSpeedNed_f());
// VECT3_DIFF(derrivative_error_to_velocity_setpoint, error_to_velocity_setpoint, previous_error_to_velocity_setpoint);
// VECT3_SMUL(derrivative_error_to_velocity_setpoint, derrivative_error_to_velocity_setpoint, running_frequency)  //unfiltered diferntiated error to setpoint

// a_wanted.x = error_to_velocity_setpoint.x * relative_position_P_GAINS.x - derrivative_error_to_velocity_setpoint.x * relative_position_D_GAINS.x;
// a_wanted.y = error_to_velocity_setpoint.y * relative_position_P_GAINS.y - derrivative_error_to_velocity_setpoint.y * relative_position_D_GAINS.y;
// a_wanted.z = error_to_velocity_setpoint.z * relative_position_P_GAINS.z - derrivative_error_to_velocity_setpoint.z * relative_position_D_GAINS.z;

a_wanted.x = error_to_setpoint_NED.x * relative_position_P_GAINS.x + relative_speed_NED.x * relative_position_D_GAINS.x;
a_wanted.y = error_to_setpoint_NED.y * relative_position_P_GAINS.y + relative_speed_NED.y * relative_position_D_GAINS.y;
a_wanted.z = error_to_setpoint_NED.z * relative_position_P_GAINS.z + relative_speed_NED.z * relative_position_D_GAINS.z;

 BoundAbs(a_wanted.x, 6);
 BoundAbs(a_wanted.y, 6);
 BoundAbs(a_wanted.z, 6);

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

  LEADER_AC_NED.x = (float) enu_pos.y *10;  // in mm
  LEADER_AC_NED.y = (float) enu_pos.x * 10;
  LEADER_AC_NED.z = (float) -enu_pos.z * 10;
  

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

  LEADER_AC_NED_SPEED.x = (float) enu_speed.y * 10;
  LEADER_AC_NED_SPEED.y = (float) enu_speed.x * 10;
  LEADER_AC_NED_SPEED.z = (float) enu_speed.z * 10;

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

extern void gps_other_ac_datalink_parse_REMOTE_GPS_SMALL(uint8_t *buf)
{
  // if (DL_REMOTE_GPS_LOCAL_ac_id(buf) != AC_ID) { return; } // not for this aircraft
  ALIVE = 1;
  parse_gps_datalink_small_2(DL_REMOTE_GPS_SMALL_heading(buf),
                            DL_REMOTE_GPS_SMALL_pos_xyz(buf),
                            DL_REMOTE_GPS_SMALL_speed_xyz(buf),
                            DL_REMOTE_GPS_SMALL_tow(buf));
}