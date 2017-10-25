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
#include "math.h"


// Global variables
static bool isInit = false;

static float Ixx = 0.00084f;
static float Ixy = 0.00000f;
static float Ixz = 0.00000f;
static float Iyy = 0.00084f;
static float Iyz = 0.00000f;
static float Izz = 0.00110f;

static float Kpq_x = 580.0f;
static float Kpq_y = 580.0f;
static float Kpq_z = 175.0f;

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

  group_id = 0;

  qdes[0] = 0.0f;
  qdes[1] = 0.0f;
  qdes[2] = 0.0f;
  qdes[3] = 1.0f;
  omg_des[0] = 0.0f;
  omg_des[1] = 0.0f;
  omg_des[2] = 0.0f;
  omg_ddes[0] = 0.0f;
  omg_ddes[1] = 0.0f;
  omg_ddes[2] = 0.0f;
  thrust_des = 0.0f;

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

  qdes[0] = d->qdes_x/1000.0f;
  qdes[1] = d->qdes_y/1000.0f;
  qdes[2] = d->qdes_z/1000.0f;
  qdes[3] = d->qdes_w/1000.0f;
  omg_des[0] = d->omg_des_x/1000.0f;
  omg_des[1] = d->omg_des_y/1000.0f;
  omg_des[2] = d->omg_des_z/1000.0f;
  omg_ddes[0] = d->omg_ddes_x/1000.0f;
  omg_ddes[1] = d->omg_ddes_y/1000.0f;
  omg_ddes[2] = d->omg_ddes_z/1000.0f;
  thrust_des = d->thrust_des/1000.0f;
}

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

  // calculate the inverse of our desired (quaternion) attitude
  float qn = 1/sqrtf(qdes[0]*qdes[0] + qdes[1]*qdes[1] + qdes[2]*qdes[2] + qdes[3]*qdes[3]);
  float qDinv[4]; // x y z w
  qDinv[0] = -qdes[0]*qn;
  qDinv[1] = -qdes[1]*qn;
  qDinv[2] = -qdes[2]*qn;
  qDinv[3] = qdes[3]*qn;

  // quaternion attitude error: perform qE = qA_skew * qDinv
  float qE[4];
  qE[0] = state->attitudeQuaternion.w*qDinv[0] + state->attitudeQuaternion.z*qDinv[1] - state->attitudeQuaternion.y*qDinv[2] + state->attitudeQuaternion.x*qDinv[3];
  qE[1] = -state->attitudeQuaternion.z*qDinv[0] + state->attitudeQuaternion.w*qDinv[1] + state->attitudeQuaternion.x*qDinv[2] + state->attitudeQuaternion.y*qDinv[3];
  qE[2] = state->attitudeQuaternion.y*qDinv[0] - state->attitudeQuaternion.x*qDinv[1] + state->attitudeQuaternion.w*qDinv[2] + state->attitudeQuaternion.z*qDinv[3];
  qE[3] = -state->attitudeQuaternion.x*qDinv[0] - state->attitudeQuaternion.y*qDinv[1] - state->attitudeQuaternion.z*qDinv[2] + state->attitudeQuaternion.w*qDinv[3];

  // just the rpy part for a vector representation
  float e_att[3];
  float dir = 1.0f;
  if(qE[3] < 0.0f)
    dir = -1.0f;
  e_att[0] = dir * qE[0];
  e_att[1] = dir * qE[1];
  e_att[2] = dir * qE[2];
  //////////////////////////////////////////////
  // calculate rotational velocity error
  //////////////////////////////////////////////

  // calculate angular velocity error
  float e_ang[3];
  float gyro[3];
  gyro[0] = sensors->gyro.x/180.0f*3.14159265f;
  gyro[1] = sensors->gyro.y/180.0f*3.14159265f;
  gyro[2] = sensors->gyro.z/180.0f*3.14159265f;
  e_ang[0] = gyro[0] - omg_des[0];
  e_ang[1] = gyro[1] - omg_des[1];
  e_ang[2] = gyro[2] - omg_des[2];

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
  term2[0] = Ixx*gyro[0] + Ixy*gyro[1] + Ixz*gyro[2];
  term2[1] = Ixy*gyro[0] + Iyy*gyro[1] + Iyz*gyro[2];
  term2[2] = Ixz*gyro[0] + Iyz*gyro[1] + Izz*gyro[2];
  taud[0] = -omg_des[2]*term2[1] + omg_des[1]*term2[2];
  taud[1] = omg_des[2]*term2[0] - omg_des[0]*term2[2];
  taud[2] = -omg_des[1]*term2[0] + omg_des[0]*term2[1];
  taud[0] += Ixx*omg_ddes[0] + Ixy*omg_ddes[1] + Ixz*omg_ddes[2];
  taud[1] += Ixy*omg_ddes[0] + Iyy*omg_ddes[1] + Iyz*omg_ddes[2];
  taud[2] += Ixz*omg_ddes[0] + Iyz*omg_ddes[1] + Izz*omg_ddes[2];

  // Body frame force control
  if(thrust_des < 0.0f)
    thrust_des = 0.0f;
  float uF = thrust_des;

  // moment control signal
  // uM = taud - J * (Kpq * e_att + Komega * e_ang);
  float uM[3];
  uM[0] = taud[0] - Ixx*(Kpq_x*e_att[0] + Komega_x*e_ang[0]) + Ixy*(Kpq_y*e_att[1] + Komega_y*e_ang[1]) + Ixz*(Kpq_z*e_att[2] + Komega_z*e_ang[2]);
  uM[1] = taud[1] - Ixy*(Kpq_x*e_att[0] + Komega_x*e_ang[0]) + Iyy*(Kpq_y*e_att[1] + Komega_y*e_ang[1]) + Iyz*(Kpq_z*e_att[2] + Komega_z*e_ang[2]);
  uM[2] = taud[2] - Ixz*(Kpq_x*e_att[0] + Komega_x*e_ang[0]) + Iyz*(Kpq_y*e_att[1] + Komega_y*e_ang[1]) + Izz*(Kpq_z*e_att[2] + Komega_z*e_ang[2]);
  //uM[0] = 0.0f;
  //uM[1] = 0.0f;
  //uM[2] = 0.0f;

  // scaling : make FM the same as the input to the CF_fcn
  float Mbx = uM[0]*(massThrust*Mscale_xy);
  float Mby = uM[1]*(massThrust*Mscale_xy);
  float Mbz = uM[2]*(massThrust*Mscale_z);

  // output control signal
  control->thrust = massThrust * uF;
  control->roll = clamp_local(Mbx, -32000, 32000);
  control->pitch = clamp_local(Mby, -32000, 32000);
  control->yaw = clamp_local(Mbz, -32000, 32000);
}

PARAM_GROUP_START(cascaded_cmd)
PARAM_ADD(PARAM_FLOAT, Ixx, &Ixx)
PARAM_ADD(PARAM_FLOAT, Ixy, &Ixy)
PARAM_ADD(PARAM_FLOAT, Ixz, &Ixz)
PARAM_ADD(PARAM_FLOAT, Iyy, &Iyy)
PARAM_ADD(PARAM_FLOAT, Iyz, &Iyz)
PARAM_ADD(PARAM_FLOAT, Izz, &Izz)
PARAM_ADD(PARAM_FLOAT, Izz, &Izz)
PARAM_ADD(PARAM_FLOAT, Kpq_x, &Kpq_x)
PARAM_ADD(PARAM_FLOAT, Kpq_y, &Kpq_y)
PARAM_ADD(PARAM_FLOAT, Kpq_z, &Kpq_z)
PARAM_ADD(PARAM_FLOAT, Komega_x, &Komega_x)
PARAM_ADD(PARAM_FLOAT, Komega_y, &Komega_y)
PARAM_ADD(PARAM_FLOAT, Komega_z, &Komega_z)
PARAM_GROUP_STOP(cascaded_cmd)

LOG_GROUP_START(cscmd_group)
LOG_ADD(LOG_UINT8, led, &group_id)
LOG_GROUP_STOP(cscmd_group)
