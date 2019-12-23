#include "MotorLib.h"



void __Set_Speed(uint8_t speed,uint8_t napr,uint8_t power){

	if(power==1){
	if(napr==0){
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
			_MotorSpeedReg3=speed;
	}
	else{
		 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
			_MotorSpeedReg3=speed;_MotorSpeedReg3=speed-speed*0.2;
	}
}
	else{
		
		     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
			_MotorSpeedReg3=0;
	}

HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);



}






