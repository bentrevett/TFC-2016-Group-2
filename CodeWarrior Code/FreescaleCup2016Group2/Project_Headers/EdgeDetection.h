/*
 * EdgeDetection.h
 *
 *  Created on: Dec 17, 2014
 *  Author: Miroslav Dobrev
 *  
 *  Header file for EdgeDetection.c
 */

#ifndef EDGEDETECTION_H_
#define EDGEDETECTION_H_

void generateKernel(void);
void edgeDetection(volatile uint16_t* input, int16_t* output, const uint32_t length);

#endif /* EDGEDETECTION_H_ */
