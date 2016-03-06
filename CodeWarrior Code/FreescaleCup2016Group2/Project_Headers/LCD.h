/*
 * LCD.h
 *
 *  Created on: Dec 8, 2015
 *      Author: bentr_000
 */

#ifndef LCD_H_
#define LCD_H_

#include "MKL25Z4.h"
#include "string.h"
#include "TFC\TFC.h"
#include "typedefs_structs\carState_s.h"
#include "main.h"

void LCDinit();
void LCDmoveCursor(int x,int y);
void LCDfeedback();
void LCDwriteString(char string[]);
void LCDwriteVariable(int variable);
void LCDwriteState(carState_s* carState);
void LCDclear();
void LCDreturn();
void itoa(int n, char s[]);
void reverse(char s[]);

#endif /* LCD_H_ */
