/*
 * main.h
 *
 *  Created on: Feb 17, 2014
 *      Author: Matt
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "TFC\TFC.h"
#include "Settings.h"

#include "derivative.h" /* include peripheral declarations */
#include "AutoExposure.h"
#include "EdgeDetection.h"
#include "LineDetectionDouble.h"
#include "SpeedControl.h"
#include "SpeedSensor.h"
#include "SteeringControl.h"
#include "TargetSpeedControl.h"
#include "ActiveDifferential.h"
#include "EdgeDetection.h"
#include "ServoMapping.h"
#include "typedefs_structs\carState_s.h"
#include "Accelerometer.h"
#include "LCD.h"
//#include "Telemetry.h"

void TFC_Init(carState_s* carState);
void TFC_Task();
int main(void);
void evaluateUARTorSpeed(carState_s* carState);
void evaluateMotorState(carState_s* carState);
void rawFocussingMode();
void derivativeFocussingMode();
void servoAlignment();;
void lineFollowingMode(carState_s* carState);
float targetSpeedAverage(float targetSpeed);
void telemetrySendData();
void LEDfeedback(carState_s* carState);
void processRaceLine(carState_s* carState);
float servoValueAverage(float servoValue);
void BluetoothSetLED(uint8_t i);
void BluetoothSetSpeed(uint8_t i);
void telemetryReadData();

#endif /* MAIN_H_ */
