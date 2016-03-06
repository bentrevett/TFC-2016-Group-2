/*
 * LCD.c
 *
 *  Created on: Dec 8, 2015
 *      Author: bentr_000
 */

#include "LCD.h"

extern carState_s carState;
extern float targetSpeed;
extern float accel;

void LCDinit(){
	uart_putchar(UART2_BASE_PTR, 'S');
	uart_putchar(UART2_BASE_PTR, 'B');
	uart_putchar(UART2_BASE_PTR, '1');
	uart_putchar(UART2_BASE_PTR, '1');
	uart_putchar(UART2_BASE_PTR, '5');
	uart_putchar(UART2_BASE_PTR, '2');
	uart_putchar(UART2_BASE_PTR, '0');
	uart_putchar(UART2_BASE_PTR, '0');
	//LCDclear();
	//LCDwriteString("CURRENT STATE:");
	//LCDmoveCursor(0,2);
	//LCDwriteString("TARGET SPEED");
	//LCDmoveCursor(0,4);
	//LCDwriteString("RIGHT WHEEL SPEED");
	//LCDmoveCursor(0,6);
	//LCDwriteString("LEFT WHEEL SPEED");
	//LCDmoveCursor(0,8);
	//LCDwriteString("ACCELEROMETER");
}

void LCDmoveCursor(int x,int y){
	uart_putchar(UART2_BASE_PTR, 'T');
	uart_putchar(UART2_BASE_PTR, 'P');
	uart_putchar(UART2_BASE_PTR, x);
	uart_putchar(UART2_BASE_PTR, y);
}

void LCDfeedback(carState_s* carState){
	//LCDinit();
	LCDclear();
	
	LCDwriteString("CURRENT STATE:");
	LCDreturn();
	//LCDmoveCursor(0,1);
	LCDwriteState(carState); 				//gives feedback for car state by the LCD display
	//LCDclear();
	LCDreturn();
	LCDwriteString("TARGET SPEED");
	//LCDwriteString("REAL-TIME FEEDBACK:");
	//LCDmoveCursor(0,3);
	LCDwriteVariable(FLOAT_TO_CHAR(LIMIT_255(8*targetSpeed)));
	LCDreturn();
	LCDwriteString("RIGHT WHEEL SPEED");
	LCDreturn();
	LCDwriteVariable(FLOAT_TO_CHAR(LIMIT_255(8*getSpeed(CHANNEL_0))));
	LCDreturn();
		LCDmoveCursor(0,6);
		LCDwriteString("LEFT WHEEL SPEED");
		LCDreturn();
	//LCDmoveCursor(0,5);
	
	//LCDmoveCursor(0,7);
		LCDwriteVariable(FLOAT_TO_CHAR(LIMIT_255(8*getSpeed(CHANNEL_1))));
		//LCDmoveCursor(15,8);
		//LCDwriteVariable(FLOAT_TO_CHAR(LIMIT_255(accel)));
	//LCDwriteVariable("Right Wheel Speed: ",);
	//LCDreturn();
	//LCDwriteVariable("Left Wheel Speed: ",FLOAT_TO_CHAR(LIMIT_255(8*getSpeed(CHANNEL_1))));
	//LCDclear();
}


void LCDwriteString(char string[]){
	
	int string_length = strlen(string);
	uart_putchar(UART2_BASE_PTR, 'T');
	uart_putchar(UART2_BASE_PTR, 'T');
	for(int i=0;i<string_length;i++){
		uart_putchar(UART2_BASE_PTR, string[i]);	
	}
	uart_putchar(UART2_BASE_PTR, 0x00);
}

void LCDwriteVariable(int variable){
	char intstring[16];
	
	itoa(variable, intstring);
	
	for(int i=0;i<strlen(intstring);i++){
		uart_putchar(UART2_BASE_PTR, 'T');
		uart_putchar(UART2_BASE_PTR, 'T');
		uart_putchar(UART2_BASE_PTR, intstring[i]);
		uart_putchar(UART2_BASE_PTR, 0x00);
	}	
}

void LCDwriteState(carState_s* carState){
	
	if(carState->lineDetectionState == STOPLINE_DETECTED)
			{
				LCDwriteString("STOPLINE DETECTED");
			}
			#ifdef CROSS_DETECTION_ENABLE
			else if(carState->crossSection == YES)
			{
				LCDwriteString("X DETECTED          ");
			}
			#endif
			else if(carState->sMode == S_MODE_ON)
			{
				LCDwriteString("S MODE ON           ");
			}
			if(carState->detectedType == DOUBLE_EDGE)
			{
				LCDwriteString("BOTH EDGES          ");
			}
			else if (carState->edge == LEFT_EDGE)
			{	
				LCDwriteString("LEFT EDGE           ");
			}
			else if (carState->edge == RIGHT_EDGE)
			{	
				LCDwriteString("RIGHT EDGE          ");
			}
			else if (carState->lineDetectionState == LINE_TEMPORARILY_LOST)
			{	
				LCDwriteString("TEMP LOST           ");
			}
			else if (carState->lineDetectionState == LINE_LOST)
			{	
				LCDwriteString("LINE LOST           ");
			}
}

void LCDclear(){

	uart_putchar(UART2_BASE_PTR, 'C');
	uart_putchar(UART2_BASE_PTR, 'L');
	uart_putchar(UART2_BASE_PTR, 'T');
		uart_putchar(UART2_BASE_PTR, 'P');
		uart_putchar(UART2_BASE_PTR, 0);
			uart_putchar(UART2_BASE_PTR, 0);
	
}

void LCDreturn(){
	
	uart_putchar(UART2_BASE_PTR, 'T');
	uart_putchar(UART2_BASE_PTR, 'R');
	uart_putchar(UART2_BASE_PTR, 'T');
}

/* itoa:  convert n to characters in s */
void itoa(int n, char s[])
{
    int i, sign;

    if ((sign = n) < 0)  /* record sign */
        n = -n;          /* make n positive */
    i = 0;
    do {       /* generate digits in reverse order */
        s[i++] = n % 10 + '0';   /* get next digit */
    } while ((n /= 10) > 0);     /* delete it */
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';
    reverse(s);
}

/* reverse:  reverse string s in place */
void reverse(char s[])
{
    int i, j;
    char c;

    for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}
