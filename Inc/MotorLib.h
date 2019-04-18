#ifndef __MOTORLIB
#define __MOTORLIB



#include "stm32f1xx_hal.h"


#define  _MotorPwmDevider 20
#define  _MotorPwmPeriod 100
#define  _JoystickMax 4096
#define  _JoystickMin 0
#define  _JoystickZero 3020
#define  _JoystickUnderZero 2900
#define  _JoystickUpToZero 3100

#define  _MotorSpeedReg TIM4->CCR4



uint8_t  __MoveForwardJoy (uint16_t speed );
uint8_t __MoveBackwardJoy  (uint16_t speed );
void MoveByJoystick (uint16_t testts);
uint8_t _MotorSpeedEnterJoy  (uint16_t speed );




#endif