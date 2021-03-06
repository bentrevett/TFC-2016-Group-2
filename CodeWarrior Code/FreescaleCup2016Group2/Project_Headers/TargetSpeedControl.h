/*
 * TargetSpeedControl.h
 *
 *  Created on: Feb 20, 2014
 *      Author: Matt and Miroslav Dobrev
 */

#ifndef TARGETSPEEDCONTROL_H_
#define TARGETSPEEDCONTROL_H_

#include "typedefs_structs\carState_s.h"
#include <stdlib.h>


float getDesiredSpeed(carState_s* carState, float speedPercent, float radiusRoot);

#endif /* TARGETSPEEDCONTROL_H_ */
