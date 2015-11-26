/*
 * ServoMapping.h
 *
 *  Created on: April 17, 2015
 *  Author: Miroslav Dobrev
 *  
 *  Header file for ServoMapping.c
 */

#ifndef SERVOMAPPING_H_
#define SERVOMAPPING_H_

#include "TFC\TFC.h"

extern float mapArray[];

float getRadiusRoot(float servoValue);
float getRadius(float servoValue);

#endif /* SERVOMAPPING_H_ */
