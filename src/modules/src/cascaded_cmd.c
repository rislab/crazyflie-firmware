/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * position_external.c: Module to receive current position and yaw from external source
 */

#include <errno.h>

/* FreeRtos includes */
#include "FreeRTOS.h"

#include "crtp.h"
#include "debug.h"
#include "num.h"
#include "configblock.h"
#include "log.h"
#include "param.h"
#include "cascaded_cmd.h"
#include <math.h>
//#include "math3d.h"


// Global variables
static bool isInit = false;
//static uint8_t my_id;
static uint8_t group_id;

static float J[3][3];
static float Ixx = 0.00084f;
static float Ixy = 0.00000f;
static float Ixz = 0.00000f;
static float Iyy = 0.00084f;
static float Iyz = 0.00000f;
static float Izz = 0.00110f;

//static float Kpq[3][3];
static float Kpq_x = 580.0f;
static float Kpq_y = 580.0f;
static float Kpq_z = 175.0f;

//static float Komega[3][3];
static float Komega_x = 36.0f;
static float Komega_y = 36.0f;
static float Komega_z = 19.0f;

static float qdes[4]; // x y z w
static float omg_des[3];
static float omg_ddes[3];
static float thrust_des;

static float massThrust = 130500;
// the "scale" values are effectively gains which
// put the new controller signals on par with the
// old controller values. Once we tune the
// controller, these should not be necessary.
static float Jscale = 10;
static float Mscale_xy = 5.0f;
static float Mscale_z = 1000.0f;

//Private functions
static void CCCrtpCB(CRTPPacket* pk);

void CascadedCmdInit(void)
{
  if(isInit) {
    return;
  }

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_CC, CCCrtpCB);

  //uint64_t address = configblockGetRadioAddress();
  //my_id = address & 0xFF;

  group_id = 0;

  J[0][0] = Ixx/Jscale;
  J[0][1] = Ixy/Jscale;
  J[0][2] = Ixz/Jscale;
  J[1][0] = Ixy/Jscale;
  J[1][1] = Iyy/Jscale;
  J[1][2] = Iyz/Jscale;
  J[2][0] = Ixz/Jscale;
  J[2][1] = Iyz/Jscale;
  J[2][2] = Izz/Jscale;


  //Kpq = diag(Kpq_x, Kpq_y, Kpq_z);
  //Komega = diag(Komega_x, Komega_y, Komega_z);


  isInit = true;
  //DEBUG_PRINT("fm. initialized: %d\n", my_id);
}

bool CascadedCmdTest(void)
{
  return isInit;
}

static void CCCrtpCB(CRTPPacket* pk)
{
  struct CCCrtpValues* d = ((struct CCCrtpValues*)pk->data);

  group_id = d->group;

  qdes[0] = d->qdes_x;
  qdes[1] = d->qdes_y;
  qdes[2] = d->qdes_z;
  qdes[3] = d->qdes_w;
  omg_des[0] = d->omg_des_x;
  omg_des[1] = d->omg_des_y;
  omg_des[2] = d->omg_des_z;
  omg_ddes[0] = d->omg_ddes_x;
  omg_ddes[1] = d->omg_ddes_y;
  omg_ddes[2] = d->omg_ddes_z;
  thrust_des = d->thrust_des;
}

/*
   void getRPMs(float* m1, float* m2, float* m3, float* m4)
   {
 *m1 = qdes_x;
 *m2 = qdes_y;
 *m3 = qdes_z;
 *m4 = qdes_w;
 }
 */

float clamp_local(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

void CascadedCmdControl(control_t *control, sensorData_t *sensors, const state_t *state)
{
  //////////////////////////////////////////////
  // calculate (quaternion) attitude error as:
  // qError = skew(qActual) * qDesired
  // and then grab the x,y,z portion of qError
  //////////////////////////////////////////////

  // get our current (quaternion) attitude
  float qAx = state->attitudeQuaternion.x;
  float qAy = state->attitudeQuaternion.y;
  float qAz = state->attitudeQuaternion.z;
  float qAw = state->attitudeQuaternion.w;

  // skew qA into a 4x4 matrix
  float qA_skew[4][4];
  // qA_skew(qA(0), -qA(1), -qA(2), -qA(3),
  //         qA(1),  qA(0),  qA(3), -qA(2),
  //         qA(2), -qA(3),  qA(0),  qA(1),
  //         qA(3),  qA(2), -qA(1),  qA(0));
  qA_skew[0][0] =  qAw;
  qA_skew[0][1] = -qAx;
  qA_skew[0][2] = -qAy;
  qA_skew[0][3] = -qAz;

  qA_skew[1][0] =  qAx;
  qA_skew[1][1] =  qAw;
  qA_skew[1][2] =  qAz;
  qA_skew[1][3] = -qAy;

  qA_skew[2][0] =  qAy;
  qA_skew[2][1] = -qAz;
  qA_skew[2][2] =  qAw;
  qA_skew[2][3] =  qAx;

  qA_skew[3][0] =  qAz;
  qA_skew[3][1] =  qAy;
  qA_skew[3][2] = -qAx;
  qA_skew[3][3] =  qAw;

  // calculate the inverse of our desired (quaternion) attitude
  float qn = 1/sqrtf(qdes[0]*qdes[0] + qdes[1]*qdes[1] + qdes[2]*qdes[2] + qdes[3]*qdes[3]);
  float qDinv[4]; // x y z w
  qDinv[0] = -qdes[0]*qn;
  qDinv[1] = -qdes[1]*qn;
  qDinv[2] = -qdes[2]*qn;
  qDinv[3] = qdes[3]*qn;

  // quaternion attitude error: perform qE = qA_skew * qDinv
  float qE[4];
  for (int row = 0; row < 4; ++row) {
    float accum = 0;
    for (int col = 0; col < 4; ++col) {
      accum += qA_skew[row][col] * qDinv[col];
    }
    qE[row] = accum;
  }

  // just the rpy part for a vector representation
  float e_att[3];
  float dir = 1.0f;
  if(qE[0] < 0.0f)
    dir = -1.0f;
  e_att[0] = dir * qE[1];
  e_att[1] = dir * qE[2];
  e_att[2] = dir * qE[3];

  //////////////////////////////////////////////
  // calculate rotational velocity error
  //////////////////////////////////////////////

  // calculate angular velocity error
  float e_ang[3];
  e_ang[0] = sensors->gyro.x - omg_des[0];
  e_ang[1] = -sensors->gyro.y - omg_des[1];
  e_ang[2] = sensors->gyro.z - omg_des[2];

  // this is the skew matrix of our desired rotational velocity
  //struct vec Srow1 = mkvec(0.0f, -omg_des.z, omg_des.y);
  //struct vec Srow2 = mkvec(omg_des.z, 0.0f, -omg_des.x);
  //struct vec Srow3 = mkvec(-omg_des.y, omg_des.x, 0.0f);
  // Somega_des( 0,          -omg_des(2),  omg_des(1),
  //             omg_des(2),  0           -omg_des(0),
  //            -omg_des(1),  omg_des(0),  0);
  //struct mat33 Somega_des = mrows(Srow1, Srow2, Srow3);

  //////////////////////////////////////////////
  // calculate control terms
  //////////////////////////////////////////////

  // calculate desired moment based on rotational
  // velocity & desired angular acceleration
  // taud = J * omg_ddes + Somega_des * J * current_angular_velocity;
  float taud[3], term2[3];
  term2[0] = J[0][0]*sensors->gyro.x + J[0][1]*(-sensors->gyro.y) + J[0][2]*sensors->gyro.z;
  term2[1] = J[1][0]*sensors->gyro.x + J[1][1]*(-sensors->gyro.y) + J[1][2]*sensors->gyro.z;
  term2[2] = J[2][0]*sensors->gyro.x + J[2][1]*(-sensors->gyro.y) + J[2][2]*sensors->gyro.z;
  taud[0] = -omg_des[2]*term2[1] + omg_des[1]*term2[2];
  taud[1] = omg_des[2]*term2[0] - omg_des[0]*term2[2];
  taud[2] = -omg_des[1]*term2[0] + omg_des[0]*term2[1];
  taud[0] += J[0][0]*omg_ddes[0] + J[0][1]*omg_ddes[1] + J[0][2]*omg_ddes[2];
  taud[1] += J[1][0]*omg_ddes[0] + J[1][1]*omg_ddes[1] + J[1][2]*omg_ddes[2];
  taud[2] += J[2][0]*omg_ddes[0] + J[2][1]*omg_ddes[1] + J[2][2]*omg_ddes[2];

  // Body frame force control
  float uF = thrust_des;
  if(thrust_des < 0.0f)
    thrust_des = 0.0f;

  // moment control signal
  // uM = taud - J * (Kpq * e_att + Komega * e_ang);
  float uM[3];
  uM[0] = taud[0] - J[0][0]*(Kpq_x*e_att[0] + Komega_x*e_ang[0]) + J[0][1]*(Kpq_y*e_att[1] + Komega_y*e_ang[1]) + J[0][2]*(Kpq_z*e_att[2] + Komega_z*e_ang[2]);
  uM[1] = taud[1] - J[0][1]*(Kpq_x*e_att[0] + Komega_x*e_ang[0]) + J[1][1]*(Kpq_y*e_att[1] + Komega_y*e_ang[1]) + J[1][2]*(Kpq_z*e_att[2] + Komega_z*e_ang[2]);
  uM[2] = taud[2] - J[0][2]*(Kpq_x*e_att[0] + Komega_x*e_ang[0]) + J[2][1]*(Kpq_y*e_att[1] + Komega_y*e_ang[1]) + J[2][2]*(Kpq_z*e_att[2] + Komega_z*e_ang[2]);

  // scaling : make FM the same as the input to the CF_fcn
  float Mbx = uM[0]*(massThrust*Jscale*Mscale_xy);
  float Mby = uM[1]*(-massThrust*Jscale*Mscale_xy);
  float Mbz = uM[2]*(massThrust*Jscale*Mscale_z);

  // output control signal
  control->thrust = massThrust * uF;
  control->roll = clamp_local(Mbx, -32000, 32000);
  control->pitch = clamp_local(Mby, -32000, 32000);
  control->yaw = clamp_local(Mbz, -32000, 32000);
}

//PARAM_GROUP_START(cascaded_cmd)
//PARAM_ADD(PARAM_FLOAT, Ixx, &Ixx)
//PARAM_ADD(PARAM_FLOAT, Ixy, &Ixy)
//PARAM_ADD(PARAM_FLOAT, Ixz, &Ixz)
//PARAM_ADD(PARAM_FLOAT, Iyy, &Iyy)
//PARAM_ADD(PARAM_FLOAT, Iyz, &Iyz)
//PARAM_ADD(PARAM_FLOAT, Izz, &Izz)
//PARAM_ADD(PARAM_FLOAT, Izz, &Izz)
//PARAM_ADD(PARAM_FLOAT, Kpq_x, &Kpq_x)
//PARAM_ADD(PARAM_FLOAT, Kpq_y, &Kpq_y)
//PARAM_ADD(PARAM_FLOAT, Kpq_z, &Kpq_z)
//PARAM_ADD(PARAM_FLOAT, Komega_x, &Komega_x)
//PARAM_ADD(PARAM_FLOAT, Komega_y, &Komega_y)
//PARAM_ADD(PARAM_FLOAT, Komega_z, &Komega_z)
//PARAM_GROUP_STOP(cascaded_cmd)

LOG_GROUP_START(cscmd_group)
LOG_ADD(LOG_UINT8, led, &group_id)
LOG_GROUP_STOP(cscmd_group)
