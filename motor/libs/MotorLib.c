#include "MotorLib.h"


void __Set_Speed(uint8_t speed,uint8_t napr,uint8_t power){

	if(power==1){
	if(napr==0){
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
	}
	else{
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
	}
}
	else{
		
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
	}
	_MotorSpeedReg4=speed;
	_MotorSpeedReg3=speed;
 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);



}










uint8_t  __MoveForwardJoy (uint16_t speed){
	
	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
	
	_MotorSpeedEnterJoy(_MotorPwmPeriod-speed/((_JoystickUnderZero-_JoystickMin)/_MotorPwmPeriod));

	return 0;
}

uint8_t __MoveBackwardJoy  (uint16_t speed){
		
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
	_MotorSpeedEnterJoy(_MotorPwmPeriod-speed/((_JoystickMax-_JoystickUpToZero)/_MotorPwmPeriod));
		//_MotorSpeedEnterJoy(_MotorPwmPeriod-speed/((_JoystickUnderZero-_JoystickMin)/_MotorPwmPeriod));
		return 0;
}

uint8_t __MoveZero  (){
		
	_MotorSpeedReg4=0;
		return 0;
}

uint8_t _MotorSpeedEnterJoy  (uint16_t speed ){
		printf("speed=%d \r\n",speed);
	_MotorSpeedReg4=speed;
 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
		return 0;
}


void MoveByJoystick (uint16_t testts){

     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
		 testts<_JoystickUnderZero ? __MoveForwardJoy(testts) :  (testts>_JoystickUpToZero ? __MoveBackwardJoy(testts) : __MoveZero());
   
}


void MoveAndTurn(uint16_t speedX,int16_t speedY){

	
	
	 //speedX<_JoystickUnderZero ? __MoveForwardJoy(speedX) :  (speedX>_JoystickUpToZero ? __MoveBackwardJoy(speedX) : __MoveZero());

	
	if(speedY<_JoystickUnderZero){
		printf("back\r\n");

		
			if(speedX<_JoystickUnderZero){
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				_MotorSpeedReg4=_MotorPwmPeriod-speedY/_YaxisUnderZero;
				_MotorSpeedReg3=_MotorPwmPeriod-(speedY+speedX)/_YaxisUnderZero ;
				//printf("speedX=%d speedY=%d \r\n",speedX,speedY);
				printf("UnderZero r=%d lt=%d \r\n",_MotorSpeedReg4,_MotorSpeedReg3);
	}
	else if (speedX>_JoystickUpToZero){
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
					_MotorSpeedReg4=_MotorPwmPeriod-speedY/_YaxisUnderZero +(_JoystickUpToZero+speedX)/_YaxisUpZero;
					_MotorSpeedReg3=_MotorPwmPeriod-speedY/_YaxisUnderZero ; 
	//	printf("speedX=%d speedY=%d \r\n",speedX,speedY);
					//printf("UpToZero r=%d l=%d \r\n",_MotorPwmPeriod-(speedY+_JoystickMax-speedX)/_YaxisUnderZero,_MotorPwmPeriod-speedY/_YaxisUnderZero );
		printf("UpToZero r=%d lt=%d \r\n",_MotorSpeedReg4,_MotorSpeedReg3);
	}
	else {
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
					_MotorSpeedReg4=_MotorPwmPeriod-speedY/((_JoystickUnderZero-_JoystickMin)/_MotorPwmPeriod);
					_MotorSpeedReg3=_MotorPwmPeriod-speedY/((_JoystickUnderZero-_JoystickMin)/_MotorPwmPeriod) ;
	//	printf("speedX=%d speedY=%d \r\n",speedX,speedY); 
    //       printf("right=%d left=%d \r\n",_MotorSpeedReg4,_MotorSpeedReg3 );
	}
	
			}
			else if(!(speedY<_JoystickUnderZero)&& !(speedY>_JoystickUpToZero)){			
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
			_MotorSpeedReg4=0;
			_MotorSpeedReg3=0;
			
			}
	
	else if(speedY>_JoystickUpToZero){ 
		printf("forward \r\n");

					if(speedX<_JoystickUnderZero){
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
				_MotorSpeedReg4=(speedY-_JoystickUpToZero)/_YaxisUpZero - speedX/(_JoystickUnderZero/100);
				_MotorSpeedReg3=(speedY-_JoystickUpToZero)/_YaxisUpZero + speedX/(_JoystickUnderZero/100) ;
				printf("speedX=%d speedY=%d \r\n",speedX,speedY);
						//printf("UnderZero r=%d lt=%d \r\n",_MotorPwmPeriod-speedY/_YaxisUpZero,_MotorPwmPeriod-(speedY+speedX)/_YaxisUpZero );
							printf("UnderZero r=%d lt=%d \r\n",_MotorSpeedReg4,_MotorSpeedReg3);
	}
	else if (speedX>_JoystickUpToZero){
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
					_MotorSpeedReg4=(speedY-_JoystickUpToZero)/_YaxisUpZero + speedX/_YaxisUpZero;
					_MotorSpeedReg3=(speedY-_JoystickUpToZero)/_YaxisUpZero - speedX/_YaxisUpZero; 
		printf("speedX=%d speedY=%d \r\n",speedX,speedY);
		//			printf("UpToZero r=%d l=%d \r\n",_MotorPwmPeriod-(speedY+_JoystickMax-speedX)/_YaxisUpZero,_MotorPwmPeriod-speedY/_YaxisUpZero );
			printf("UpToZero r=%d lt=%d \r\n",_MotorSpeedReg4,_MotorSpeedReg3);
	}
	else {
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
					_MotorSpeedReg4=_MotorPwmPeriod-speedY/_YaxisUpZero;
					_MotorSpeedReg3=_MotorPwmPeriod-speedY/_YaxisUpZero ;
	//	printf("speedX=%d speedY=%d \r\n",speedX,speedY); 
   //        printf("right=%d left=%d \r\n",_MotorSpeedReg4,_MotorSpeedReg3 );
	}
		
		
	}
				else if(!(speedY<_JoystickUnderZero)&& !(speedY>_JoystickUpToZero)){			
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
			_MotorSpeedReg4=0;
			_MotorSpeedReg3=0;
			
			}

	
	

	
	
	
}


