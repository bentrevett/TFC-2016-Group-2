/*
 * SpeedControl.c
 *
 *  Created on: Feb 3, 2014
 *      Author: Matt
 */

#include <SpeedControl.h>

#define KP 0.3f				// Matt Original: 0.3f
#define KI 0.5f				// Matt Original: 0.5f
//#define KD 0.0f			// Matt: disabled in the code

#define MAX_PWM 1.0f
#define MIN_PWM -0.8f

#define INCLUDE_INTEGRAL 1
#define NO_INTEGRAL 0

float getDesiredMotorPWM(float setpoint, float measurement, volatile isNewMeasurementAvailable_t* isNewMeasurementAvailable, uint8_t channel)
{
	static struct persistantPIDVariables_s PIDVariables[] = {
	{ 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 } };

	if (*isNewMeasurementAvailable == NEW_MEASUREMENT_AVAILABLE)
	{
		*isNewMeasurementAvailable = NO_NEW_MEASUREMENT_AVAILABLE;

		float dt = (TFC_Ticker[2] / 10000.0f);
		TFC_Ticker[2] = 0;
		if(dt > 0.1f)
		{
			dt = 0.1f;
		}
		
		// Integral wind-up protection
		if (PIDVariables[channel].PWM < MAX_PWM && PIDVariables[channel].PWM > MIN_PWM)
		{
			PID(setpoint, measurement, &PIDVariables[channel], INCLUDE_INTEGRAL, dt);
		}
		else
		{
			PID(setpoint, measurement, &PIDVariables[channel], NO_INTEGRAL, dt);
		}

		
		if (PIDVariables[channel].PWM < MIN_PWM)
			PIDVariables[channel].PWM = MIN_PWM;
		else if (PIDVariables[channel].PWM > MAX_PWM)
			PIDVariables[channel].PWM = MAX_PWM;
	}
	return PIDVariables[channel].PWM;
}

void PID(float setpoint, float measurement, struct persistantPIDVariables_s* PIDVariables, uint8_t incrementIntegral, float dt)
{
	PIDVariables->error = setpoint - measurement;
	PIDVariables->errorSum += PIDVariables->error * incrementIntegral * dt;
	PIDVariables->PWM = (PIDVariables->error * KP) + (PIDVariables->errorSum * KI); /* + (PIDVariables->dError * KD); */
}

