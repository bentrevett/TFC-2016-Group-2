#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#include "I2C.h"
#include "TFC\TFC.h"

void Init_Accelerometer();
void zeroAccelerometer();
float getXAcc();
float getYAcc();
float getZAcc();
int16_t getRawXAcc();
int16_t getRawYAcc();
int16_t getRawZAcc();
uint8_t getID();


#endif /* ACCELEROMETER_H_ */
