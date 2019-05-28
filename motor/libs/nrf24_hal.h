#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H


// Hardware abstraction layer for NRF24L01+ transceiver (hardware depended functions)
// GPIO pins definition
// GPIO pins initialization and control functions
// SPI transmit functions


// Peripheral libraries
#include "main.h"

extern SPI_HandleTypeDef hspi2;

// SPI port peripheral
#define nRF24_SPI_PORT             hspi2

// nRF24 GPIO peripherals
//#define nRF24_GPIO_PERIPHERALS     (RCC_APB2ENR_IOPBEN)

// CE (chip enable) pin (PB11)
#define nRF24_CE_PORT              GPIOA
#define nRF24_CE_PIN               GPIO_PIN_6
#define nRF24_CE_L()               HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)
#define nRF24_CE_H()               HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)

// CSN (chip select negative) pin (PA4)
#define nRF24_CSN_PORT             GPIOA
#define nRF24_CSN_PIN              GPIO_PIN_5
#define nRF24_CSN_L()              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define nRF24_CSN_H()              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)

// IRQ pin (PB10)
#define nRF24_IRQ_PORT             GPIOA
#define nRF24_IRQ_PIN              GPIO_PIN_7


// Function prototypes
void nRF24_GPIO_Init(void);
uint8_t nRF24_LL_RW(uint8_t data);

#endif // __NRF24_HAL_H
