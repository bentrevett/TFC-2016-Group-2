/*
 * ActiveDifferential.c
 *
 *  Created on: Feb 21, 2014
 *      Author: Matt and Miroslav Dobrev
 */

#include "ActiveDifferential.h"
#include "Settings.h"
#include "ServoMapping.h"

#define WHEELS_HALF_WIDTH	0.0685f		// (13.7/2) cm from centre of wheel to centre of car in meters

float getActiveDifferentialModifier(float servoValue, int8_t channel)
{
#ifdef ACTIVE_MAP_ENABLE
	float radius = getRadius(servoValue);
	int8_t turnDirection = (servoValue > 0.0f) ? 1 : -1;
	
	if(channel == 0)
	{
		return (K_ACTIVE_COMP*(radius + turnDirection*WHEELS_HALF_WIDTH)/radius);
	}
	else
	{
		return (K_ACTIVE_COMP*(radius - turnDirection*WHEELS_HALF_WIDTH)/radius);
	}
#else
	if(channel == 0)
	{
		return (1.0f + (servoValue*K_ACTIVE));
	}
	else
	{
		return (1.0f - (servoValue*K_ACTIVE));
	}
#endif
}

