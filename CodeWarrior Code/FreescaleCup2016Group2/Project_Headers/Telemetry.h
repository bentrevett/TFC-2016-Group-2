/*
 * Telemetry.h
 *
 *  Created on: Jan 26, 2016
 *      Author: bentr_000
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_


#include "main.h"
#include "AutoExposure.h"
#include "typedefs_structs\carState_s.h"

void telemetrySendData();
void cameraFeed(int camera_number);
void BluetoothSetLED(uint8_t i);
void BluetoothSetSpeed(uint8_t i);
void telemetryReadData();

#endif /* TELEMETRY_H_ */
