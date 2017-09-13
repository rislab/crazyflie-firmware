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
#include "fm.h"


// Global variables
static bool isInit = false;
//static uint8_t my_id;
static uint8_t group_id;

static float mixer_inv[4][4];

//No idea what these numbers should be
static float length = 0.005f;
static float mscale = 1.0f;

static float m1_rpm = 0.0f;
static float m2_rpm = 0.0f;
static float m3_rpm = 0.0f;
static float m4_rpm = 0.0f;

//Private functions
static void FMCrtpCB(CRTPPacket* pk);

void FMInit(void)
{
  if(isInit) {
    return;
  }

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_FM, FMCrtpCB);

  isInit = true;

  //uint64_t address = configblockGetRadioAddress();
  //my_id = address & 0xFF;

  // This is the uninverted mixer.  Will eventually have to find the real numbers
  mixer_inv[0][0] = 1.f;
  mixer_inv[0][1] = 0.f;
  mixer_inv[0][2] = 0.f;
  mixer_inv[0][3] = 0.f;

  mixer_inv[1][0] = 0.f;
  mixer_inv[1][1] = 1.f;
  mixer_inv[1][2] = 0.f;
  mixer_inv[1][3] = 0.f;

  mixer_inv[2][0] = 0.f;
  mixer_inv[2][1] = 0.f;
  mixer_inv[2][2] = 1.f;
  mixer_inv[2][3] = 0.f;

  mixer_inv[3][0] = 0.f;
  mixer_inv[3][1] = 0.f;
  mixer_inv[3][2] = 0.f;
  mixer_inv[3][3] = 1.f;


  group_id = 0;

  //DEBUG_PRINT("fm. initialized: %d\n", my_id);
}

bool FMTest(void)
{
  return isInit;
}

static void FMCrtpCB(CRTPPacket* pk)
{
  struct FMCrtpValues* d = ((struct FMCrtpValues*)pk->data);

  m1_rpm = mixer_inv[0][0]*d->F + mixer_inv[0][1]*d->Mx + mixer_inv[0][2]*d->My + mixer_inv[0][3]*d->Mz;
  m2_rpm = mixer_inv[1][0]*d->F + mixer_inv[1][1]*d->Mx + mixer_inv[1][2]*d->My + mixer_inv[1][3]*d->Mz;
  m3_rpm = mixer_inv[2][0]*d->F + mixer_inv[2][1]*d->Mx + mixer_inv[2][2]*d->My + mixer_inv[2][3]*d->Mz;
  m4_rpm = mixer_inv[3][0]*d->F + mixer_inv[3][1]*d->Mx + mixer_inv[3][2]*d->My + mixer_inv[3][3]*d->Mz;

  group_id = d->group;
}

void getRPMs(float* m1, float* m2, float* m3, float* m4)
{
  *m1 = m1_rpm;
  *m2 = m2_rpm;
  *m3 = m3_rpm;
  *m4 = m4_rpm;
}

PARAM_GROUP_START(fm)
PARAM_ADD(PARAM_FLOAT, length, &length)
PARAM_ADD(PARAM_FLOAT, mscale, &mscale)
PARAM_GROUP_STOP(fm)

LOG_GROUP_START(fm_group)
LOG_ADD(LOG_UINT8, led, &group_id)
LOG_GROUP_STOP(fm_group)
