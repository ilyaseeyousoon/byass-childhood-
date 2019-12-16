#ifndef __MOTORLIB
#define __MOTORLIB



#include "stm32f1xx_hal.h"


//#define  _MotorPwmDevider 2
//#define  _MotorPwmPeriod 100
//#define  _JoystickMax 4096
//#define  _JoystickMin 0
//#define  _JoystickZero 3070
//#define  _JoystickUnderZero 2900
//#define  _JoystickUpToZero 3200


#define  _MotorPwmDevider 250
#define  _MotorPwmPeriod 100
#define  _JoystickMax 4096
#define  _JoystickMin 0
#define  _JoystickZero 3070
#define  _JoystickUnderZero 2900
#define  _JoystickUpToZero 3200

#define  _YaxisUnderZero  ((_JoystickUnderZero-_JoystickMin)/_MotorPwmPeriod)
#define  _YaxisUpZero   ((_JoystickMax-_JoystickUpToZero)/_MotorPwmPeriod)

#define  _MotorSpeedReg3 TIM3->CCR2

extern  TIM_HandleTypeDef htim3;




void __Set_Speed(uint8_t speed,uint8_t napr,uint8_t power);

#endif