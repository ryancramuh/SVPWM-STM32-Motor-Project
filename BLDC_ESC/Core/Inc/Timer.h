/*
 * Timer.h
 *
 *  Created on: Feb 14, 2025
 *      Author: peterh17
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#ifdef _TIMER_C
   #define SCOPE
#else
   #define SCOPE extern
#endif

#define NUMBER_OF_TIMERS			4

#define UPDATE_PWM_TIMER				0
#define RECALCULATE_PWM_UPDATE_TIMER	1
#define RECALCULATE_PWM_UPDATE_TIME		100
#define STARTUP_TIMER 2
#define UPDATE_LCD_TIMER 3
#define UPDATE_LCD_TIME 100

SCOPE unsigned short sTimer[NUMBER_OF_TIMERS];

SCOPE void TIMER2_HANDLE(void);

#undef SCOPE
#endif /* INC_TIMER_H_ */
