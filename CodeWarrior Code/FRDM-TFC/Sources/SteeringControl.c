/*
 * SteeringControl.c
 *
 *  Created on: Nov 14, 2013
 *      Author: Matt
 *      Modified By: Miroslav Dobrev
 */

#include "TFC\TFC.h"
#include "Settings.h"

#define NEW_SERVO_RESCALE 0.8f	// 0.7f Rescale to accommodate for new servo arm length

float getDesiredServoValue(int8_t position, int8_t setpoint, lineScanState_t* lineScanState)
{
	static float newPosition = 0;
	static float errorSum = 0;
	static float previousError = 0;

	if (*lineScanState == LINESCAN_IMAGE_READY)
	{
		*lineScanState = NO_NEW_LINESCAN_IMAGE;
		float error = (float) (setpoint - position);

		if (abs(newPosition) < STEERING_LIMIT_UPPER)
		{
			errorSum += error * (TFC_Ticker[1] / 10000.0f);
		}

		if (errorSum > INTEGRAL_LIMIT)
		{
			errorSum = INTEGRAL_LIMIT;
		}
		else if (errorSum < -INTEGRAL_LIMIT)
		{
			errorSum = -INTEGRAL_LIMIT;
		}

		float errorDifferential = (error - previousError) / (TFC_Ticker[1] / 10000.0f);
		TFC_Ticker[1] = 0;

		newPosition = (Kp * error) + (Ki * errorSum) + (Kd * errorDifferential);
		
		
		
		if (newPosition > STEERING_LIMIT_UPPER)
		{
			newPosition = STEERING_LIMIT_UPPER;
		}
		else if (newPosition < STEERING_LIMIT_LOWER)
		{
			newPosition = STEERING_LIMIT_LOWER;
		}
		
		previousError = error;
		
		#ifdef NEW_SERVO_RESCALE
		newPosition = newPosition*NEW_SERVO_RESCALE;
		#endif
	}
	
	return SERVO_MOUNT_DIRECTION*newPosition;
}
