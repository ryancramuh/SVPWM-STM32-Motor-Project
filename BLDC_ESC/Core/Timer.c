/*
 * Timer.c
 *
 *  Created on: Feb 14, 2025
 *      Author: peterh17
 */

#define _TIMER_C

#include "TIMER.h"

void TIMER2_HANDLE(void)
{
	unsigned short sIndex;
	//__disable_irq();

	for (sIndex=0; sIndex<NUMBER_OF_TIMERS; sIndex++)
	{
		if (sTimer[sIndex] != 0)
			sTimer[sIndex]--;
	}
	//__enable_irq();
}

