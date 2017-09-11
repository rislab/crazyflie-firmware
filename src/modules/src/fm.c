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
#include "fm.h"


// Global variables
static bool isInit = false;
static uint8_t my_id;
static uint8_t group_id;

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

  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;

  group_id = 0;

  DEBUG_PRINT("fm. initialized: %d\n", my_id);
}

bool FMTest(void)
{
  return isInit;
}

static void FMCrtpCB(CRTPPacket* pk)
{
  struct FMCrtpValues* d = ((struct FMCrtpValues*)pk->data);

  group_id = d->group;
}

LOG_GROUP_START(fm_group)
LOG_ADD(LOG_UINT8, led, &group_id)
LOG_GROUP_STOP(fm_group)

