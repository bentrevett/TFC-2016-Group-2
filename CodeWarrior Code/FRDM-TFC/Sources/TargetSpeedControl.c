/*
 * TargetSpeedControl.c
 *
 *  Created on: Feb 20, 2014
 *      Author: Matt and Miroslav Dobrev
 */

#include "TargetSpeedControl.h"
#include "Settings.h"

extern float friction_correct;

float getDesiredSpeed(carState_s* carState, float speedPercent, float radiusRoot)
{
	float speed;
	float minSpeed = MIN_SPEED;
	float maxSpeed = MAX_SPEED;
	
	if(carState->carMode == STRAIGHT_MODE)
	{
		maxSpeed = MAX_STRAIGHT_SPEED;
	}
	
	if(abs(carState->raceLineCenter) > ERROR_THRESHOLD)
	{
		#ifdef SPEED_MAP_ENABLE
		speed = (FRICTION_COEFF_ROOT + friction_correct)*radiusRoot*20.0f; // sqrt(fric_coef*g*R) rescaled by 20/PI to convert to car speed units
																		   // simplified as sqrt(g) = PI (approx)
		#else
		speed = (speedPercent * maxSpeed * (1 - (K_LIN * abs(carState->raceLineCenter))));
		#endif
	}
	else
	{
		speed = speedPercent*maxSpeed;
	}
	
	if(speed > (speedPercent*maxSpeed))
		speed = (speedPercent*maxSpeed);
	else if(speed < minSpeed)
		speed = minSpeed;
	
	return speed;
}
