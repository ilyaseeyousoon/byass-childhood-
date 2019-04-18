#ifndef __MOTORLIB
#define __MOTORLIB

#include "stm32f1xx_hal.h"


#define  _RgbLedPwmDevider 20
#define  _RgbLedPwmPeriod 255


#define RgbLedRedReg 		TIM4->CCR1
#define RgbLedRedGreen	TIM4->CCR2
#define RgbLedRedBlue		TIM4->CCR3


void RgbLedSet(uint16_t Red,uint16_t Green,uint16_t Blue);










#endif