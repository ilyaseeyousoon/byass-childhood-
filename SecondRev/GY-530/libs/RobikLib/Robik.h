#ifndef __ROBIK_H
#define __ROBIK_H


#include "ranging_vl53l0x.h"
#include "vl53l0x_api.h"
#include "nrf24.h"
#include "nrf24_hal.h"

#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF

enum module
{
	raspbery,
  joySteak,
  motor,
  rgbLed,  
	distanceSensor
};

    static const uint8_t nRF24_ADDR0[] = { 0x01,0x01,0xE7, 0x1F, 0xE1};
		static const uint8_t nRF24_ADDR1[] = {  0x01,0x01,0xE7, 0x1F, 0xE2 };
		static const uint8_t nRF24_ADDR2[] = {  0x01,0x01,0xE7, 0x1F, 0xE3 };
		static const uint8_t nRF24_ADDR3[] = {  0x01,0x01,0xE7, 0x1F, 0xE4 };
		static const uint8_t nRF24_ADDR4[] = {  0x01,0x01,0xE7, 0x1F, 0xE5 };
		static const uint8_t nRF24_ADDR5[] = {  0x01,0x01,0xE7, 0x1F, 0xE6 };
		
extern uint8_t payload_length;
extern uint8_t identNumber ;
		
typedef enum {
	nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
	nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
	nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
	nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;



nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length);

void TransmitComand (uint8_t comandIdent,uint8_t tempArr[payload_length]);

void TransmitIdentification ();


void SerialNumbFunc();

void OnAction();


void NRF_tuning();









#endif /* __ROBIK_H */
