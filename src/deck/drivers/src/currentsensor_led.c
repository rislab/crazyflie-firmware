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
 * currentsensor.c: current sensor driver
 */

#define DEBUG_MODULE "CURRENT_SENSOR"

#include "FreeRTOS.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "task.h"

#include "currentsensor.h"
#include "ledring12.h"
#include "ws2812.h"

static bool isInit;
float current = 0;
float capacity = 0;


void currentsensorInit(DeckInfo* info)
{
  if (isInit)
    return;

  adcInit();
  xTaskCreate(currentsensorTask, "CURRENTSENSE", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
  isInit = true;
  DEBUG_PRINT("INIT\n");
  DEBUG_PRINT("ID: %s\n",CURRENT_SENSOR_ID);
  DEBUG_PRINT("%f A/V\n",CURRENT_SENSE_AMPS_PER_VOLT);
  DEBUG_PRINT("Offset: %f\n",CURRENT_SENSE_OFFSET);

}

void currentsensorTask(void* arg)
{
  systemWaitStart();
  TickType_t xLastWakeTime;
  //uint16_t adctest = 0;

  int vbatid = logGetVarId("pm", "vbat");
  float vbat, vfrac;
  float discharged_current = 0;

  while (1) {
    xLastWakeTime = xTaskGetTickCount();
    current = currentSensorAmps();
    //adctest = analogRead(CURRENT_SENSE_PIN);
    //DEBUG_PRINT("%f\t%d\n",current,adctest);

    vbat = logGetFloat(vbatid);
    vfrac = (vbat - MIN_VOLTAGE)/(MAX_VOLTAGE - MIN_VOLTAGE);
    if(vfrac > capacity){
      capacity = vfrac;
      discharged_current = (1.0 - capacity) * TOTAL_CAPACITY;
    }else{
      discharged_current -= current * 0.1;
      capacity = discharged_current / TOTAL_CAPACITY;
    }

    vTaskDelayUntil(&xLastWakeTime, M2T(100)); // wait 100ms
  }
}

bool currentsensorTest(void)
{
  bool testStatus = true;

  if (!isInit)
    return false;

  DEBUG_PRINT("Test [%d]\n",testStatus);

  return testStatus;
}

float currentSensorAmps(){

  float voltage, current;

  voltage = analogRead(CURRENT_SENSE_PIN) * 3.3 / adcRange;
  //DEBUG_PRINT("Voltage: %f\n",voltage);
  current = CURRENT_SENSE_AMPS_PER_VOLT * (voltage - CURRENT_SENSE_OFFSET);

  return current;
}

// TODO: Decide on vid:pid and set the used pins
static const DeckDriver currentsensor_deck = {
  .vid = 0, // Changed this from 0
  .pid = 0, // Changed this from 0
  .name = "CurrentSensor",
  .usedGpio = 0,

  .init = currentsensorInit,
  .test = currentsensorTest,
};

DECK_DRIVER(currentsensor_deck);

LOG_GROUP_START(currentsensor)
LOG_ADD(LOG_FLOAT, current, &current)
LOG_ADD(LOG_FLOAT, capacity, &capacity)
LOG_GROUP_STOP(currentsensor)
