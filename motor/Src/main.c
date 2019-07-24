/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MotorLib.h"
#include "nrf24.h"
#include "nrf24_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#include <stdbool.h>

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t global_help=0;
uint8_t err=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

    static const uint8_t nRF24_ADDR0[] = { 0x01,0x01,0xE7, 0x1C, 0xE1};
		static const uint8_t nRF24_ADDR1[] = {  0x01,0x01,0xE7, 0x1C, 0xE2 };
		static const uint8_t nRF24_ADDR2[] = {  0x01,0x01,0xE7, 0x1C, 0xE3 };
		static const uint8_t nRF24_ADDR3[] = {  0x01,0x01,0xE7, 0x1C, 0xE4 };
		static const uint8_t nRF24_ADDR4[] = {  0x01,0x01,0xE7, 0x1C, 0xE5 };
		static const uint8_t nRF24_ADDR5[] = {  0x01,0x01,0xE7, 0x1C, 0xE6 };
		
	uint32_t	cw=0;
			uint8_t SerialModule[4];
		
		
		uint16_t dataArr[30]={0};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}


const uint8_t payload_length=10;

uint32_t i,j,k;
uint8_t monitoreFlag=0;
// Buffer to store a payload of maximum width
uint8_t nRF24_payload[10]={0};

// Pipe number
nRF24_RXResult pipe;
uint8_t pipe1;
// Length of received payload

// Helpers for transmit mode demo

// Timeout counter (depends on the CPU speed)
// Used for not stuck waiting for IRQ
#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF

// Result of packet transmission
typedef enum {
	nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
	nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
	nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
	nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;

nRF24_TXResult tx_res;






// Function to transmit data packet
// input:
//   pBuf - pointer to the buffer with data to transmit
//   length - length of the data buffer in bytes
// return: one of nRF24_TX_xx values
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
	
	volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	nRF24_CE_L();

	// Transfer a data from the specified buffer to the TX FIFO
	nRF24_WritePayload(pBuf, length);

	// Start a transmission by asserting CE pin (must be held at least 10us)
	nRF24_CE_H();

	// Poll the transceiver status register until one of the following flags will be set:
	//   TX_DS  - means the packet has been transmitted
	//   MAX_RT - means the maximum number of TX retransmits happened
	// note: this solution is far from perfect, better to use IRQ instead of polling the status
	do {
		status = nRF24_GetStatus();
		if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
			break;
		}
	} while (wait--);

	// Deassert the CE pin (Standby-II --> Standby-I)
	nRF24_CE_L();

	if (!wait) {
		// Timeout
		return nRF24_TX_TIMEOUT;
	}

	// Check the flags in STATUS register
	printf("[");
	printf("%d",status);
	printf("] ");

	// Clear pending IRQ flags
    nRF24_ClearIRQFlags();

	if (status & nRF24_FLAG_MAX_RT) {
		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
		return nRF24_TX_MAXRT;
	}

	if (status & nRF24_FLAG_TX_DS) {
		// Successful transmission
		return nRF24_TX_SUCCESS;
	}

	// Some banana happens, a payload remains in the TX FIFO, flush it
	nRF24_FlushTX();

	return nRF24_TX_ERROR;
}
//1- iamhere 2- return data valid
void TransmitComand (uint8_t comandIdent,uint8_t tempArr[payload_length]){

	uint8_t Ident[]={2,100,100,100,100,100,100,100,100,100};

	Ident[1]=SerialModule[0];
	Ident[2]=SerialModule[1];
	Ident[3]=comandIdent;
	
	switch(comandIdent){
		case 1:
		//nop
	
		break;
		
		case 2:
		Ident[4]=tempArr[4];
		Ident[5]=tempArr[5];
		Ident[6]=tempArr[6];
		
		break;
	}
			nRF24_SetPowerMode(nRF24_PWR_DOWN);
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);
    nRF24_SetOperationalMode(nRF24_MODE_TX);
    nRF24_ClearIRQFlags();
	  nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR2);
    nRF24_SetPowerMode(nRF24_PWR_UP);	
	    
	    tx_res = nRF24_TransmitPacket(Ident, payload_length);// data load
	
//	switch (tx_res) {
//			case nRF24_TX_SUCCESS:
//				printf("OK");
//				break;
//			case nRF24_TX_TIMEOUT:
//				printf("TIMEOUT");
//				break;
//			case nRF24_TX_MAXRT:
//				printf("MAX RETRANSMIT");
//				break;
//			default:
//				printf("ERROR");
//				break;
//		}
//    	printf("\r\n");
		
	
	
	
			nRF24_SetPowerMode(nRF24_PWR_DOWN);
    nRF24_SetOperationalMode(nRF24_MODE_RX);
    nRF24_SetPowerMode(nRF24_PWR_UP);
    nRF24_CE_H();
}









/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

		volatile uint32_t *UniqueID = (uint32_t *)0x1FFFF7E8;
	volatile uint32_t __UniqueID[3];
	__UniqueID[0] = UniqueID[0];
	__UniqueID[1] = UniqueID[1];
	__UniqueID[2] = UniqueID[2];

	SerialModule[0]=__UniqueID[2];
	SerialModule[1]=__UniqueID[2]>>8;
	SerialModule[2]=__UniqueID[2]>>16;
	SerialModule[3]=__UniqueID[2]>>24;

	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
 printf("\r\nSTM32F103RET6 is online.\r\n");

    // RX/TX disabled
    nRF24_CE_L();
		
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		    // Configure the nRF24L01+
    printf("nRF24L01+ check: ");
		
    while (!nRF24_Check()) {
    	printf("FAIL\r\n");
			HAL_Delay(1000);
    }
		
	printf("OK\r\n");
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		  // Initialize the nRF24L01 to its default state
    nRF24_Init();
		
		
		
	// This is simple receiver with multiple RX pipes:
    //   - pipe#0 address: "WBC"
    //   - pipe#0 payload: 11 bytes
	//   - pipe#1 address: '0xE7 0x1C 0xE3'
    //   - pipe#1 payload: 5 bytes
    //   - pipe#4 address: '0xE7 0x1C 0xE6' (this is pipe#1 address with different last byte)
    //   - pipe#4 payload: 32 bytes (the maximum payload length)
	//   - RF channel: 115 (2515MHz)
	//   - data rate: 250kbps (minimum possible, to increase reception reliability)
	//   - CRC scheme: 2 byte

    // The transmitter sends packets of different length to the three different logical addresses,
    // cycling them one after another, that packets comes to different pipes (0, 1 and 4)

    // Disable ShockBurst for all RX pipes
    nRF24_DisableAA(0xFF);

    // Set RF channel
    nRF24_SetRFChannel(115);

    // Set data rate
    nRF24_SetDataRate(nRF24_DR_1Mbps);

    // Set CRC scheme
    nRF24_SetCRCScheme(nRF24_CRC_2byte);

    // Set address width, its common for all pipes (RX and TX)
    nRF24_SetAddrWidth(5);

    // Configure RX PIPE#0

    nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR0); // program address for RX pipe #0
		nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR1); // program address for RX pipe #1
		nRF24_SetAddr(nRF24_PIPE2, nRF24_ADDR2); // program address for RX pipe #2
		nRF24_SetAddr(nRF24_PIPE3, nRF24_ADDR3); // program address for RX pipe #3
		nRF24_SetAddr(nRF24_PIPE4, nRF24_ADDR4); // program address for RX pipe #4
		nRF24_SetAddr(nRF24_PIPE5, nRF24_ADDR5); // program address for RX pipe #5
    //nRF24_SetRXPipe(nRF24_PIPE0, nRF24_AA_OFF, 6); 
		nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, payload_length); 
		//nRF24_SetRXPipe(nRF24_PIPE2, nRF24_AA_OFF, 6); 
		//nRF24_SetRXPipe(nRF24_PIPE3, nRF24_AA_OFF, 6); 
		//nRF24_SetRXPipe(nRF24_PIPE4, nRF24_AA_OFF, 6); 
		//nRF24_SetRXPipe(nRF24_PIPE5, nRF24_AA_OFF, 6); 


    // Set operational mode (PRX == receiver)
    nRF24_SetOperationalMode(nRF24_MODE_RX);

    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);

    // Put the transceiver to the RX mode
    nRF24_CE_H();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t m=0;
  while (1)
  {

    	if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) { 
    		pipe = nRF24_ReadPayload(nRF24_payload, payload_length);

			nRF24_ClearIRQFlags();

					dataArr[m]=nRF24_payload[2];
				m++;
				if(nRF24_payload[3]==0){
			__Set_Speed(nRF24_payload[4],nRF24_payload[5],nRF24_payload[6]);	
        nRF24_FlushRX();
        nRF24_FlushTX();			
				}				
				if(nRF24_payload[3]==3){
__Set_Speed(nRF24_payload[4],nRF24_payload[5],nRF24_payload[6]);
//TransmitComand(2,nRF24_payload);
				}
//TransmitOK(nRF24_payload);
//MoveAndTurn(nRF24_payload[2]*256+nRF24_payload[3],nRF24_payload[4]*256+nRF24_payload[5]);

//						printf(" PAYLOAD:>");
//			printf("%d,motor=%d,%d",nRF24_payload[0]*256+nRF24_payload[1],
//				nRF24_payload[2]*256+nRF24_payload[3],
//				nRF24_payload[4]*256+nRF24_payload[5]);
//			printf("\r\n");
				
    	}
			else{
	if(monitoreFlag==1){
			//	TransmitIdentification();
		//TransmitComand(1,nRF24_payload);
		monitoreFlag=0;
	}
}
			
			HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = _MotorPwmDevider;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = _MotorPwmPeriod;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 256000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_11 
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA11 
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_11 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
