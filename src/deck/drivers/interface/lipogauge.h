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
 * lipogauge.h: Lipo Gauge sensor driver
 */

#ifndef _LIPO_GAUGE_H_
#define _LIPO_GAUGE_H_

#define LIPO_GAUGE_DEFAULT_ADDRESS 0b0110110

// 12-bit Battery Voltage Measurement
#define LIPO_GAUGE_VCELL_HIGH 0x02
#define LIPO_GAUGE_VCELL_LOW 0x03

// 16-bit SOC 
#define LIPO_GAUGE_SOC_HIGH 0x04
#define LIPO_GAUGE_SOC_LOW 0x05

// Version
#define LIPO_GAUGE_VERSION_HIGH 0x08
#define LIPO_GAUGE_VERSION_LOW 0x09

// Config
#define LIPO_GAUGE_CONFIG_HIGH 0x0C
#define LIPO_GAUGE_CONFIG_LOW 0x0D

/** Default constructor, uses external I2C address.
 * @see LIPO_GAUGE_DEFAULT_ADDRESS
 */
void lipogaugeInit(DeckInfo* info);

bool lipogaugeTest(void);
void lipogaugeWorker(void* data);
void lipogaugeTask(void* arg);

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool lipogaugeTestConnection();

/** Get Version
 * This register is used to verify the version number of the device
 * @return Model ID
 * @see VL53L0X_RA_IDENTIFICATION_MODEL_ID
 */
uint16_t MAX17043GetVersion();

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool lipogaugeInitSensor();

#endif 
