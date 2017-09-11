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
 * lipogauge.c: Lipo Fuel Gauge sensor driver
 */

#define DEBUG_MODULE "LIPO_GAUGE"

#include "FreeRTOS.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "timers.h"
#include "worker.h"
#include "task.h"

#include "i2cdev.h"
#include "lipogauge.h"


static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit;

// Convert SOC register output to percentage
static float lipogaugeSOCPercent();

static uint16_t lipogaugeReadReg16Bit(uint8_t reg);
//static bool lipogaugeWriteReg16Bit(uint8_t reg, uint16_t val);

static float SOC_percent;
//static xTimerHandle timer;

/** Default constructor, uses default I2C address.
 * @see LIPO_GAUGE_DEFAULT_ADDRESS
 */

/*static void lipoTimer(xTimerHandle timer)
{
  workerSchedule(lipogaugeWorker, NULL);

}

void lipogaugeWorker(void* data)
{

	SOC_percent = lipogaugeSOCPercent();  // get SOC percentage
	DEBUG_PRINT("%.6f\n",SOC_percent);

}*/


void lipogaugeInit(DeckInfo* info)
{
	if (isInit)
		return;

	i2cdevInit(I2C1_DEV);
	I2Cx = I2C1_DEV;
	devAddr = LIPO_GAUGE_DEFAULT_ADDRESS;
	/*timer = xTimerCreate( "LipoGaugeTimer", M2T(2000),
                                     pdTRUE, NULL, lipoTimer );
 	 xTimerStart(timer, 100);*/
	xTaskCreate(lipogaugeTask, "LIPOGAUGE", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	isInit = true;
	DEBUG_PRINT("INIT\n");
}

void lipogaugeTask(void* arg)

{
	systemWaitStart();
	TickType_t xLastWakeTime;
	while (1) {
		xLastWakeTime = xTaskGetTickCount();
		SOC_percent = 0;
		SOC_percent = lipogaugeSOCPercent();  // get SOC percentage
		//SOC_percent = 12.3456789;
		DEBUG_PRINT("%f\n",(double)SOC_percent);
		vTaskDelayUntil(&xLastWakeTime, M2T(3000)); // wait 100ms
	}
}

bool lipogaugeTest(void)
{
	bool testStatus = true;

	if (!isInit)
		return false;

	DEBUG_PRINT("TEST\n");
	//testStatus  = lipogaugeTestConnection();
	//testStatus &= lipogaugeInitSensor();

	return testStatus;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool lipogaugeTestConnection()
{
	bool ret = true;
	ret &= MAX17043GetVersion() == 3;
	return ret;
}

/** Get Version.
 * This register is used to verify the version number of the device
 * @see LIPO_GAUGE_VERSION_HIGH
 */
uint16_t MAX17043GetVersion()
{
	return lipogaugeReadReg16Bit(LIPO_GAUGE_VERSION_HIGH);
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool lipogaugeInitSensor()
{

	return true;
}

// Convert SOC register output to percentage
float lipogaugeSOCPercent()
{
	float SOC_percentage = 0;
	float SOC_low_byte = 0;
	uint8_t output = 0;

	i2cdevReadByte(I2Cx, devAddr, LIPO_GAUGE_SOC_HIGH, &output); // read high byte
	SOC_percentage = output / 1.0; // low byte has units %

	i2cdevReadByte(I2Cx, devAddr, LIPO_GAUGE_SOC_LOW, &output); // read low byte
	SOC_low_byte = output / 256.0; // low byte has units 1/256%

	SOC_percentage += SOC_low_byte; // get full percentage value

	return SOC_percentage;
}

uint16_t lipogaugeReadReg16Bit(uint8_t reg)
{
	uint8_t buffer[2] = {};
	i2cdevRead(I2Cx, devAddr, reg, 2, (uint8_t *)&buffer);
	return ((uint16_t)(buffer[0]) << 8) | buffer[1];
}

/*bool lipogaugeWriteReg16Bit(uint8_t reg, uint16_t val)
{
	uint8_t buffer[2] = {};
	buffer[0] = ((val >> 8) & 0xFF);
	buffer[1] = ((val     ) & 0xFF);
	return i2cdevWrite(I2Cx, devAddr, reg, 2, (uint8_t *)&buffer);
}*/

// TODO: Decide on vid:pid and set the used pins
static const DeckDriver lipogauge_deck = {
	.vid = 0, // Changed this from 0
	.pid = 0, // Changed this from 0
	.name = "LipoGauge",
	.usedGpio = 0,

	.init = lipogaugeInit,
	.test = lipogaugeTest,
};

DECK_DRIVER(lipogauge_deck);

LOG_GROUP_START(lipogauge)
LOG_ADD(LOG_FLOAT, SOC, &SOC_percent)
LOG_GROUP_STOP(lipogauge)
