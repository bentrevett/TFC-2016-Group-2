/*
 * Telemetry.c
 *
 *  Created on: Jan 26, 2016
 *      Author: bentr_000
 */

#include "Telemetry.h"

#define BLUETOOTH_FLAG	-128

extern int32_t newExposure;
extern float servoValue;
extern carState_s carState;
extern uint16_t loop_time;
extern float targetSpeed;
extern float maxSpeedPercent;

uint8_t received_byte1 = 0; //received byte from bluetooth 0 to 255
uint8_t received_byte2 = 0;
uint8_t count = 0; //bluetooth count

void cameraFeed(int camera_number){
	
	uart_putchar(UART2_BASE_PTR, BLUETOOTH_FLAG);
	
	for(int i=0;i<128;i++){
			uart_putchar(UART2_BASE_PTR, (LineScanImage0Buffer[camera_number][i]/100));
		}	
}

void telemetrySendData()
{
	uart_putchar(UART2_BASE_PTR, BLUETOOTH_FLAG);
	
	// 1 - Track Centre Detected by the Camera
	uart_putchar(UART2_BASE_PTR, (signed char)carState.lineCenter);
	// 2 - Calculated Target Speed
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(LIMIT_255(8*targetSpeed)));
	// 3 - Speed of the right wheel
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(LIMIT_255(8*getSpeed(CHANNEL_0))));
	// 4 - Speed of the left wheel
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(LIMIT_255(8*getSpeed(CHANNEL_1))));
	// 5 - PWM output to right motor
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(126*TFC_GetMotorPWM(CHANNEL_0)));
	// 6 - PWM output to left motor
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(126*TFC_GetMotorPWM(CHANNEL_1)));
	// 7 - Current of the right motor
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(LIMIT_255(60*TFC_ReadMotorCurrent(CHANNEL_0))));
	// 8 - Current of the left motor
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(LIMIT_255(60*TFC_ReadMotorCurrent(CHANNEL_1))));
	// 9 - Servo Value
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(126*servoValue));
	// 10 - Detected edge type - Left:100, Right:-100, Track: 0, None:50
	uart_putchar(UART2_BASE_PTR, (signed char)(carState.edge==NO_EDGE)?(50):((carState.edge==BOTH_EDGE)?(0):((carState.edge==LEFT_EDGE)?(100):(-100))));
	// 11 - Camera Exposure Time, 1 unit = 0.1 ms
	uart_putchar(UART2_BASE_PTR, FLOAT_TO_CHAR(LIMIT_255((float)newExposure*0.025f)));
	// 12 - Loop time
	uart_putchar(UART2_BASE_PTR, (signed char)loop_time);
	// 13 - Line Distance
	uart_putchar(UART2_BASE_PTR, (signed char)carState.lineDistance);
}

void telemetryReadData(){
	
	if(count==0){
		//if something is on the UART, get it and put it in received_byte
		if(uart_getchar_present(UART2_BASE_PTR)){
			received_byte1 = uart_getchar(UART2_BASE_PTR);
			GPIOB_PSOR |= (1<<8);
			GPIOB_PSOR |= (1<<9);
			GPIOB_PSOR |= (1<<10);
			GPIOB_PSOR |= (1<<11);
			count++;
		}
	}
	else if(count>0){
		if(uart_getchar_present(UART2_BASE_PTR)){
			received_byte2 = uart_getchar(UART2_BASE_PTR);
			count=0;
		}
	}
	
	if(received_byte1!=0 && received_byte2!=0){
		switch(received_byte1){
		case 255: //SPEED
			BluetoothSetSpeed(received_byte2);
			break;
		case 251: //LED
			BluetoothSetLED(received_byte2);
			break;
		}
		//received_byte1=0;
		//received_byte2=0;
	}
	
}

void BluetoothSetSpeed(uint8_t i){
	maxSpeedPercent=i;
		if(maxSpeedPercent>100){
			maxSpeedPercent=100;
		}
}

void BluetoothSetLED(uint8_t i){	
		switch(i){
			case 1: 
				GPIOB_PSOR |= (1<<8);
				GPIOB_PCOR |= (1<<9);
				GPIOB_PCOR |= (1<<10);
				GPIOB_PCOR |= (1<<11);
				break;
			case 2:
				GPIOB_PCOR |= (1<<8);
				GPIOB_PSOR |= (1<<9);
				GPIOB_PCOR |= (1<<10);
				GPIOB_PCOR |= (1<<11);
				break;
			case 3:
				GPIOB_PCOR |= (1<<8);
				GPIOB_PCOR |= (1<<9);
				GPIOB_PSOR |= (1<<10);
				GPIOB_PCOR |= (1<<11);			
				break;
			case 4: 
				GPIOB_PCOR |= (1<<8);
				GPIOB_PCOR |= (1<<9);
				GPIOB_PCOR |= (1<<10);
				GPIOB_PSOR |= (1<<11);
				break;
			default:
				GPIOB_PCOR |= (1<<8);
				GPIOB_PCOR |= (1<<9);
				GPIOB_PCOR |= (1<<10);
				GPIOB_PCOR |= (1<<11);
			}
}
