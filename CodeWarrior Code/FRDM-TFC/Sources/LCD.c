/*
 * LCD.c
 *
 *  Created on: Dec 8, 2015
 *      Author: bentr_000
 */

#include "LCD.h"
#include "TFC\TFC.h"

void LCDinit(){
	
	uart_putchar(UART2_BASE_PTR, 'S');
	uart_putchar(UART2_BASE_PTR, 'B');
	uart_putchar(UART2_BASE_PTR, '1');
	uart_putchar(UART2_BASE_PTR, '1');
	uart_putchar(UART2_BASE_PTR, '5');
	uart_putchar(UART2_BASE_PTR, '2');
	uart_putchar(UART2_BASE_PTR, '0');
	uart_putchar(UART2_BASE_PTR, '0');
	uart_putchar(UART2_BASE_PTR, 0x00);
	
	//TFC_InitUARTs(115200, 115200);
}

void LCDwriteString(char string[]){
	
	uart_putchar(UART2_BASE_PTR, 'C');
	uart_putchar(UART2_BASE_PTR, 'L');
	
	for(int i=0;i<strlen(string);i++){
		uart_putchar(UART2_BASE_PTR, 'T');
		uart_putchar(UART2_BASE_PTR, 'T');
		uart_putchar(UART2_BASE_PTR, string[i]);
		uart_putchar(UART2_BASE_PTR, 0x00);
	}
}

void LCDwriteVariable(char string[],int variable){
	
	uart_putchar(UART2_BASE_PTR, 'C');
	uart_putchar(UART2_BASE_PTR, 'L');
	
	LCDwriteString(string);
	
	uart_putchar(UART2_BASE_PTR, 'T');
	uart_putchar(UART2_BASE_PTR, 'P');
	uart_putchar(UART2_BASE_PTR, 0); //x
	uart_putchar(UART2_BASE_PTR, 1); //y
	
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
	
	//uart_putchar(UART2_BASE_PTR, 'C');
	//uart_putchar(UART2_BASE_PTR, 'L');
	
	if(carState->lineDetectionState == STOPLINE_DETECTED)	// Also set in lineFollowingMode()
			{
				LCDwriteString("STOPLINE DETECTED");
			}
			#ifdef CROSS_DETECTION_ENABLE
			else if(carState->crossSection == YES)
			{
				LCDwriteString("CROSS SECTION DETECTED");
			}
			#endif
			else if(carState->sMode == S_MODE_ON)
			{
				LCDwriteString("S_MODE ACTIVATED");
			}
			if(carState->detectedType == DOUBLE_EDGE)
			{
				LCDwriteString("BOTH EDGES FOUND");
			}
			else if (carState->edge == LEFT_EDGE)
			{	
				LCDwriteString("LEFT EDGE FOUND");
			}
			else if (carState->edge == RIGHT_EDGE)
			{	
				LCDwriteString("RIGHT EDGE FOUND");
			}
			else if (carState->lineDetectionState == LINE_TEMPORARILY_LOST)
			{	
				LCDwriteString("LINE TEMP LOST");
			}
			else if (carState->lineDetectionState == LINE_LOST)
			{	
				LCDwriteString("LINE LOST :(");
			}
			
	uart_putchar(UART2_BASE_PTR, 'C');
	uart_putchar(UART2_BASE_PTR, 'S');
	uart_putchar(UART2_BASE_PTR, 0);
			
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
