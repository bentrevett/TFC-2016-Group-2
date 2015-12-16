/*
 * carState_s.h
 *
 *  Created on: Feb 18, 2014
 *      Author: Matt and Miroslav Dobrev
 */

#ifndef CARSTATE_S_H_
#define CARSTATE_S_H_

#include <stdint.h>

typedef enum {FORCED_DISABLED, RUNNING} motorState_t;
typedef enum {SINGLE_SPEED_SINGLE_UART, DUAL_SPEED_NO_UART, UNDEFINED} UARTSpeedState_t;
typedef enum {LINE_FOUND, LINE_TEMPORARILY_LOST, STOPLINE_DETECTED, LINE_LOST} lineDetectionState_t;
typedef enum {LINESCAN_IMAGE_READY, NO_NEW_LINESCAN_IMAGE} lineScanState_t;

typedef enum {NONE, SINGLE_EDGE, DOUBLE_EDGE} detectedType_t;
typedef enum {NO_EDGE, LEFT_EDGE, RIGHT_EDGE, BOTH_EDGE} edge_t;

typedef enum {NO, YES} crossSection_t;
typedef enum {NO_CROSS, BEGINNING, MIDDLE} crossSectionPosition_t;

typedef enum {NORMAL_MODE, STRAIGHT_MODE} carMode_t;
typedef enum {S_MODE_OFF, S_MODE_ON} sMode_t;

typedef struct {
	motorState_t motorState;
	UARTSpeedState_t UARTSpeedState;
	lineDetectionState_t lineDetectionState;
	lineScanState_t lineScanState;
	
	detectedType_t detectedType;
	edge_t edge;
	
	crossSection_t crossSection;
	crossSectionPosition_t crossSectionPosition;
	
	carMode_t carMode;
	
	sMode_t sMode;
	
	int8_t lineCenter;
	int8_t raceLineCenter;
	uint8_t lineDistance;
	
}carState_s;

#endif /* CARSTATE_S_H_ */
