#include "TFC\TFC.h"

static volatile unsigned int DelayTimerTick = 0;
volatile uint32_t TFC_Ticker[NUM_TFC_TICKERS];


//Since this SysTick is part of the Cortex M0 Core, you need to look in the
//Cortex M0 Generic users Guide

//TFC_Ticker[0] - used for servo update
//TFC_Ticker[1] - used for steering control derivative dt measurement
//TFC_Ticker[2] - used for speed control integral dt measurement
//TFC_Ticker[3] - used for line loss "watchdog" timer
//TFC_Ticker[4] - used for telemetry send
//TFC_Ticker[5] - used for general timing (loop timing, stopline timing, etc.)
//TFC_Ticker[6] - used for battery level measurement
//TFC_Ticker[7] - used for S-mode activation
//TFC_TIcker[8] - used for accelerometer


//See Section 4.4
void TFC_InitSysTick()
{
	uint8_t i;
	
	for(i=0;i<NUM_TFC_TICKERS;i++)
		TFC_Ticker[i] = 0;
	
	SYST_RVR = CORE_CLOCK/SYSTICK_FREQUENCY;
	SYST_CSR = SysTick_CSR_ENABLE_MASK | SysTick_CSR_TICKINT_MASK | SysTick_CSR_CLKSOURCE_MASK;

	//Important!  Since the Systick is part of the Cortex core and NOT a kinetis peripheral
	// its Interrupt line is not passed through NVIC.   You need to make sure that
	//the SysTickIRQ function is poopulated in the vector table.  See the kinetis_sysinit.c file
}

void SysTick_Handler()
{
	uint8_t i;

	if(DelayTimerTick<0xFFFFFFFF)
	{
		DelayTimerTick++;
	}
	
	for(i=0;i<NUM_TFC_TICKERS;i++)
		if(TFC_Ticker[i]<0xFFFFFFFF)
			TFC_Ticker[i]++;
}

void TFC_Delay_mS(unsigned int TicksIn_mS)
{
	DelayTimerTick = 0;

	while(DelayTimerTick<TicksIn_mS)
	{
	}
}

