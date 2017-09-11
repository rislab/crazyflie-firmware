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
 * currentsensor.h: current sensor driver
 */

#ifndef _CURRENT_SENSOR_DECK_H_
#define _CURRENT_SENSOR_DECK_H_

#define CURRENT_SENSE_PIN 11


#define CURRENT_SENSOR_ID "ACS722"
#define CURRENT_SENSE_AMPS_PER_VOLT 3.7878f
#define CURRENT_SENSE_OFFSET 1.5f

#define TOTAL_CAPACITY 600.0f
#define MAX_VOLTAGE 4.2f
#define MIN_VOLTAGE 3.0f

static uint32_t adcRange = 1 << 12;

void currentsensorInit(DeckInfo* info);

bool currentsensorTest(void);
void currentsensorTask(void* arg);
float currentSensorAmps(void);

#endif
