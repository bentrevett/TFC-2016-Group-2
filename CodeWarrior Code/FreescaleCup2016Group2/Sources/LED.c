/*
 * LED.c
 *
 *  Created on: Jan 26, 2016
 *      Author: bentr_000
 */

#include "LED.h"

extern carState_s carState;
extern float batteryLevel;

void LEDfeedback(carState_s* carState)
{
	if(batteryLevel > LOW_BATTERY)
	{
		if(carState->lineDetectionState == STOPLINE_DETECTED)	// Also set in lineFollowingMode()
		{
			GPIOB_PCOR |= (1<<8);
			GPIOB_PCOR |= (1<<9);
			GPIOB_PSOR |= (1<<10);
			GPIOB_PSOR |= (1<<11);
		}
		#ifdef CROSS_DETECTION_ENABLE
		else if(carState->crossSection == YES)
		{
			GPIOB_PSOR |= (1<<8);
			GPIOB_PCOR |= (1<<9);
			GPIOB_PCOR |= (1<<10);
			GPIOB_PSOR |= (1<<11);
		}
		#endif
		else if(carState->sMode == S_MODE_ON)
		{
			GPIOB_PCOR |= (1<<8);
			GPIOB_PSOR |= (1<<9);
			GPIOB_PSOR |= (1<<10);
			GPIOB_PCOR |= (1<<11);
		}
		else if(carState->detectedType == DOUBLE_EDGE)
		{
			GPIOB_PSOR |= (1<<8);
			GPIOB_PSOR |= (1<<9);
			GPIOB_PCOR |= (1<<10);
			GPIOB_PCOR |= (1<<11);
		}
		else if (carState->edge == LEFT_EDGE)
		{	
			GPIOB_PSOR |= (1<<8);
			GPIOB_PCOR |= (1<<9);
			GPIOB_PCOR |= (1<<10);
			GPIOB_PCOR |= (1<<11);
		}
		else if (carState->edge == RIGHT_EDGE)
		{	
			GPIOB_PCOR |= (1<<8);
			GPIOB_PSOR |= (1<<9);
			GPIOB_PCOR |= (1<<10);
			GPIOB_PCOR |= (1<<11);
		}
		else if (carState->lineDetectionState == LINE_TEMPORARILY_LOST)
		{	
			GPIOB_PCOR |= (1<<8);
			GPIOB_PCOR |= (1<<9);
			GPIOB_PSOR |= (1<<10);
			GPIOB_PCOR |= (1<<11);
		}
		else if (carState->lineDetectionState == LINE_LOST)
		{	
			GPIOB_PCOR |= (1<<8);
			GPIOB_PCOR |= (1<<9);
			GPIOB_PCOR |= (1<<10);
			GPIOB_PSOR |= (1<<11);
		}
	}
	else
	{
		GPIOB_PSOR |= (1<<8);
		GPIOB_PSOR |= (1<<9);
		GPIOB_PSOR |= (1<<10);
		GPIOB_PSOR |= (1<<11);		
	}
}
