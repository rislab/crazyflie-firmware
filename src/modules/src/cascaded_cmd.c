/** *    ||          ____  _ __
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
#include "l1_att_observer.h"
#include "median_filter.h"
#include "math.h"

#define M_PI_F ((float) M_PI)

static uint8_t group_id;

// Global variables
static bool isInit = false;

// Assume diagonal intertia matrix
static float Ixx = 0.00084f;
//static float Ixy = 0.00000f;
//static float Ixz = 0.00000f;
static float Iyy = 0.00084f;
//static float Iyz = 0.00000f;
static float Izz = 0.00110f;

//static float Kpq_x = 580.0f;
//static float Kpq_y = 580.0f;
//static float Kpq_z = 175.0f;
static float Kpq_x = 9.0f;
static float Kpq_y = 9.0f;
static float Kpq_z = 8.0f;

//static float Komega_x = 36.0f;
//static float Komega_y = 36.0f;
//static float Komega_z = 19.0f;
static float Komega_x = 1.62f;
static float Komega_y = 1.62f;
static float Komega_z = 1.64049f;

static float qdes[4]; // x y z w
static float omg_des[3];
static float omg_ddes[3];
static float thrust_des;
static float vicon_yaw;

//static float massThrust = 130500.0f;
static float massThrust = 1.0f;
// the "scale" values are effectively gains which
// put the new controller signals on par with the
// old controller values. Once we tune the
// controller, these should not be necessary.
//static float Mscale_xy = 5.0f;
//static float Mscale_z = 1000.0f;
static float Mscale_xy = 1.0f;
static float Mscale_z = 1.0f;

static float uM[3];

//Private functions
static void CCCrtpCB(CRTPPacket* pk);
static void CCGainsCB(CRTPPacket* pk);

void CascadedCmdInit(void)
{
  if(isInit) {
    return;
  }

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_CC, CCCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_CCGAINS, CCGainsCB);

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
  vicon_yaw = 0.0f;
  thrust_des = 0.0f;

  L1AttObserverSetParameters(Ixx, Iyy, Izz, Mscale_xy, Mscale_z, massThrust);
  MedianFilterInit();

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
  vicon_yaw = d->heading/1000.0f;
  thrust_des = d->thrust_des/1000.0f;
}

static void CCGainsCB(CRTPPacket* pk)
{
  struct crtpCCGains* d = ((struct crtpCCGains*)pk->data);

  Kpq_x = d->Kpq_x;
  Kpq_y = d->Kpq_y;
  Kpq_z = d->Kpq_z;

  Komega_x = d->Komega_x/1000.0f;
  Komega_y = d->Komega_y/1000.0f;
  Komega_z = d->Komega_z/1000.0f;
}

float clamp_local(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

void CascadedCmdControl(fm_t *fm, sensorData_t *sensors, const state_t *state)
{
  //////////////////////////////////////////////
  // calculate (quaternion) attitude error as:
  // qError = skew(qActual) * qDesired
  // and then grab the x,y,z portion of qError
  //////////////////////////////////////////////

  //float cr = cos(state->attitude.roll/180.0f*M_PI_F), sr = sin(state->attitude.roll/180.0f*M_PI_F);
  //float cp = cos(state->attitude.pitch/180.0f*M_PI_F), sp = sin(state->attitude.pitch/180.0f*M_PI_F);
  //float cy = cos(state->attitude.yaw/180.0f*M_PI_F), sy = sin(state->attitude.yaw/180.0f*M_PI_F);
  //qx = cy * sr * cp - sy * cr * sp;
  //qy = cy * cr * sp + sy * sr * cp;
  //qz = sy * cr * cp - cy * sr * sp;
  //qw = cy * cr * cp + sy * sr * sp;

  //float gx = 2 * (qdes[1] * qdes[3] - qdes[0] * qdes[2]);
  //float gy = 2 * (qdes[0] * qdes[1] + qdes[2] * qdes[3]);
  //float gz = qdes[0] * qdes[0] - qdes[1] * qdes[1] - qdes[2] * qdes[2] + qdes[3] * qdes[3];
  //float yaw_des = atan2f(2*(qdes[0]*qdes[3] + qdes[1]*qdes[2]), qdes[0]*qdes[0] + qdes[1]*qdes[1] - qdes[2]*qdes[2] - qdes[3]*qdes[3]) * 180 / M_PI_F;
  //float pitch_des = asinf(gx) * 180 / M_PI_F; //Pitch seems to be inverted
  //float roll_des = atan2f(gy, gz);

  //e_att2[0] = state->attitude.roll/180.0f*M_PI_F - roll_des;
  //e_att2[1] = state->attitude.pitch/180.0f*M_PI_F - pitch_des;
  //e_att2[2] = vicon_yaw - yaw_des;

  float yaw_adjust = state->attitude.yaw/180.0f*M_PI_F - vicon_yaw;

  // qx and qy == 0.0f
  float vicon_qz = sin(yaw_adjust*0.5f);
  float vicon_qw = cos(yaw_adjust*0.5f);

  // Quaternion mult given qx,qy == 0.0f
  float adjusted_qdes[4];
  adjusted_qdes[0] = vicon_qw*qdes[0] - vicon_qz*qdes[1];
  adjusted_qdes[1] = vicon_qw*qdes[1] + vicon_qz*qdes[0];
  adjusted_qdes[2] = vicon_qw*qdes[2] + vicon_qz*qdes[3];
  adjusted_qdes[3] = vicon_qw*qdes[3] - vicon_qz*qdes[2];

  // calculate the inverse of our desired (quaternion) attitude
  float qn = 1/sqrtf(adjusted_qdes[0]*adjusted_qdes[0] + adjusted_qdes[1]*adjusted_qdes[1] + adjusted_qdes[2]*adjusted_qdes[2] + adjusted_qdes[3]*adjusted_qdes[3]);
  float qDinv[4]; // x y z w
  qDinv[0] = -adjusted_qdes[0]*qn;
  qDinv[1] = -adjusted_qdes[1]*qn;
  qDinv[2] = -adjusted_qdes[2]*qn;
  qDinv[3] = adjusted_qdes[3]*qn;

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
  e_att[0] = 2.0f * dir * qE[0];
  e_att[1] = -2.0f * dir * qE[1]; // pitch flip
  e_att[2] = 2.0f * dir * qE[2];
  //////////////////////////////////////////////
  // calculate rotational velocity error
  //////////////////////////////////////////////

  // calculate angular velocity error
  float e_ang[3];
  float gyro[3];
  gyro[0] = sensors->gyro.x/180.0f*M_PI_F;
  gyro[1] = -sensors->gyro.y/180.0f*M_PI_F; // pitch flip
  gyro[2] = sensors->gyro.z/180.0f*M_PI_F;

  //MedianFilterUpdate(gyro[0], gyro[1], gyro[2]);
  //MedianFilterQuery(&omega[0], &omega[1], &omega[2]);

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
  //term2[0] = Ixx*gyro[0] + Ixy*gyro[1] + Ixz*gyro[2];
  //term2[1] = Ixy*gyro[0] + Iyy*gyro[1] + Iyz*gyro[2];
  //term2[2] = Ixz*gyro[0] + Iyz*gyro[1] + Izz*gyro[2];
  term2[0] = Ixx*gyro[0];
  term2[1] = Iyy*gyro[1];
  term2[2] = Izz*gyro[2];
  taud[0] = -omg_des[2]*term2[1] + omg_des[1]*term2[2];
  taud[1] = omg_des[2]*term2[0] - omg_des[0]*term2[2];
  taud[2] = -omg_des[1]*term2[0] + omg_des[0]*term2[1];
  //taud[0] += Ixx*omg_ddes[0] + Ixy*omg_ddes[1] + Ixz*omg_ddes[2];
  //taud[1] += Ixy*omg_ddes[0] + Iyy*omg_ddes[1] + Iyz*omg_ddes[2];
  //taud[2] += Ixz*omg_ddes[0] + Iyz*omg_ddes[1] + Izz*omg_ddes[2];
  taud[0] += Ixx*omg_ddes[0];
  taud[1] += Iyy*omg_ddes[1];
  taud[2] += Izz*omg_ddes[2];

  // Body frame force control
  if(thrust_des < 0.0f)
    thrust_des = 0.0f;
  //float uF = thrust_des;
  fm->thrust = thrust_des;

  // moment control signal
  // uM = taud - J * (Kpq * e_att + Komega * e_ang);
  //float uM[3];
  //uM[0] = taud[0] - Ixx*(Kpq_x*e_att[0] + Komega_x*e_ang[0]) + Ixy*(Kpq_y*e_att[1] + Komega_y*e_ang[1]) + Ixz*(Kpq_z*e_att[2] + Komega_z*e_ang[2]);
  //uM[1] = taud[1] - Ixy*(Kpq_x*e_att[0] + Komega_x*e_ang[0]) + Iyy*(Kpq_y*e_att[1] + Komega_y*e_ang[1]) + Iyz*(Kpq_z*e_att[2] + Komega_z*e_ang[2]);
  //uM[2] = taud[2] - Ixz*(Kpq_x*e_att[0] + Komega_x*e_ang[0]) + Iyz*(Kpq_y*e_att[1] + Komega_y*e_ang[1]) + Izz*(Kpq_z*e_att[2] + Komega_z*e_ang[2]);
  uM[0] = taud[0] - Ixx*(Kpq_x*e_att[0] + Komega_x*e_ang[0]);
  uM[1] = taud[1] - Iyy*(Kpq_y*e_att[1] + Komega_y*e_ang[1]);
  uM[2] = taud[2] + Izz*(Kpq_z*e_att[2] + Komega_z*e_ang[2]);

  fm->moment_x = uM[0];
  fm->moment_y = uM[1];
  fm->moment_z = uM[2];
  //fm->moment_x = 0.0f;
  //fm->moment_y = 0.0f;
  //fm->moment_z = 0.0f;

  // scaling : make FM the same as the input to the CF_fcn
  //float Mbx = uM[0]*(massThrust*Mscale_xy);
  //float Mby = uM[1]*(massThrust*Mscale_xy);
  //float Mbz = uM[2]*(massThrust*Mscale_z);

  // output control signal
  //control->thrust = massThrust * uF;
  //control->roll = clamp_local(Mbx, -32000, 32000);
  //control->pitch = clamp_local(Mby, -32000, 32000);
  //control->yaw = clamp_local(Mbz, -32000, 32000);

  // L1 Adaptation
  L1AttObserverUpdate(fm, sensors, 1.0f/RATE_MAIN_LOOP);
  L1AttObserverApply(uM);

  // scaling : make FM the same as the input to the CF_fcn
  fm->moment_x = uM[0];
  fm->moment_y = uM[1];
  fm->moment_z = uM[2];
}

PARAM_GROUP_START(cascaded_cmd)
PARAM_ADD(PARAM_FLOAT, Ixx, &Ixx)
  //PARAM_ADD(PARAM_FLOAT, Ixy, &Ixy)
  //PARAM_ADD(PARAM_FLOAT, Ixz, &Ixz)
PARAM_ADD(PARAM_FLOAT, Iyy, &Iyy)
  //PARAM_ADD(PARAM_FLOAT, Iyz, &Iyz)
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
LOG_ADD(LOG_FLOAT, moment_x, &uM[0])
LOG_ADD(LOG_FLOAT, moment_y, &uM[1])
LOG_ADD(LOG_FLOAT, moment_z, &uM[2])
LOG_ADD(LOG_FLOAT, Kpq_x, &Kpq_x)
LOG_ADD(LOG_FLOAT, Kpq_y, &Kpq_y)
LOG_ADD(LOG_FLOAT, Kpq_z, &Kpq_z)
LOG_ADD(LOG_FLOAT, Komega_x, &Komega_x)
LOG_ADD(LOG_FLOAT, Komega_y, &Komega_y)
LOG_ADD(LOG_FLOAT, Komega_z, &Komega_z)
LOG_GROUP_STOP(cscmd_group)
