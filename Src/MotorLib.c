#include "MotorLib.h"




uint8_t  __MoveForwardJoy (uint16_t speed){
	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
	
	_MotorSpeedEnterJoy(_MotorPwmPeriod-speed/((_JoystickUnderZero-_JoystickMin)/_MotorPwmPeriod));

	return 0;
}

uint8_t __MoveBackwardJoy  (uint16_t speed){
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
	_MotorSpeedEnterJoy(speed/((_JoystickMax-_JoystickUpToZero)/_MotorPwmPeriod));
		return 0;
}

uint8_t _MotorSpeedEnterJoy  (uint16_t speed ){
	
	_MotorSpeedReg=speed;

		return 0;
}


void MoveByJoystick (uint16_t testts){


	testts<_JoystickUnderZero ? __MoveForwardJoy(testts) :  (testts>_JoystickUpToZero ? __MoveBackwardJoy(testts) : _MotorSpeedEnterJoy(0));
   
}




