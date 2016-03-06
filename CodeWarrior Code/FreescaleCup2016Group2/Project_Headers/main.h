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
#include "LED.h"
#include "Telemetry.h"

#define CHANNEL_0 0
#define CHANNEL_1 1
#define PUSH_BUTTON_1 1
#define PUSH_BUTTON_2 2

#define FLOAT_TO_CHAR(x)	(x >= 0.0) ? (signed char)(x + 0.5) : (signed char)(x - 0.5)		
#define LIMIT_255(x)		(x <= 254.5f && x >= 0.0f && !(x >= 127.5f && x <= 128.5f)) ? x : ((x > 255.0f) ? 255.0f : ((x >= 127.5f && x <= 128.5f) ? 127.0f : 0.0f))
#define LIMIT_ABS_127(x)	(x <= 126.9f && x >= -126.9f) ? x : ((x > 126.9f) ? 126.9f : -126.9f)

#define LINE_SCAN_IMAGE(x)	LINE_SCAN_IMAGE_EXAPND(x)	// Second level of indirection needed to expand x defined as macro
#define LINE_SCAN_IMAGE_EXAPND(x)	LineScanImage##x

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
void LEDfeedback(carState_s* carState);
void processRaceLine(carState_s* carState);
float servoValueAverage(float servoValue);

#endif /* MAIN_H_ */
