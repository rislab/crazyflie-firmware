/**
 *    ||          ____  _ __  ______
 * +------+      / __ )(_) /_/ ____/_________ _____  ___
 * | 0xBC |     / __  / / __/ /    / ___/ __ `/_  / / _	\
 * +------+    / /_/ / / /_/ /___ / /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\____//_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 */

#ifndef CASCADED_CMD_H_
#define CASCADED_CMD_H_
#include <stdint.h>
#include <stdbool.h>
#include "stabilizer_types.h"

/**
 * CRTP CASCADED_CMD data struct
 */
struct CCCrtpValues {
  uint8_t group;
  int16_t qdes_x;
  int16_t qdes_y;
  int16_t qdes_z;
  int16_t qdes_w;
  int16_t omg_des_x;
  int16_t omg_des_y;
  int16_t omg_des_z;
  int16_t omg_ddes_x;
  int16_t omg_ddes_y;
  int16_t omg_ddes_z;
  int16_t heading;
  int16_t thrust_des;
} __attribute__((packed));

struct crtpCCGains {
  uint16_t Kpq_x;
  uint16_t Kpq_y;
  uint16_t Kpq_z;
  uint16_t Komega_x;
  uint16_t Komega_y;
  uint16_t Komega_z;
} __attribute__((packed));

void CascadedCmdInit(void);
bool CascadedCmdTest(void);
// void getRPMs(float* m1, float* m2, float* m3, float* m4);
void CascadedCmdControl(fm_t *fm, sensorData_t *sensors, const state_t *state);

#endif /* CASCADED_CMD_H_ */
