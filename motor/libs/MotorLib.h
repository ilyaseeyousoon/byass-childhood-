#ifndef __MOTORLIB
#define __MOTORLIB



#include "stm32f1xx_hal.h"


#define  _MotorPwmDevider 2
#define  _MotorPwmPeriod 100
#define  _JoystickMax 4096
#define  _JoystickMin 0
#define  _JoystickZero 3070
#define  _JoystickUnderZero 2900
#define  _JoystickUpToZero 3200
#define  _YaxisUnderZero  ((_JoystickUnderZero-_JoystickMin)/_MotorPwmPeriod)
#define  _YaxisUpZero   ((_JoystickMax-_JoystickUpToZero)/_MotorPwmPeriod)
#define  _MotorSpeedReg4 TIM4->CCR4
#define  _MotorSpeedReg3 TIM4->CCR3
extern  TIM_HandleTypeDef htim4;


uint8_t  __MoveForwardJoy (uint16_t speed );
uint8_t __MoveBackwardJoy  (uint16_t speed );
void MoveByJoystick (uint16_t testts);
uint8_t _MotorSpeedEnterJoy  (uint16_t speed );
uint8_t __MoveZero  ();
void MoveAndTurn(uint16_t speedX,int16_t speedY);


#endif