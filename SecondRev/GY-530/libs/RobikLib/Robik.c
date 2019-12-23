#include "Robik.h"


			uint8_t payload_length=10;
			uint8_t SerialModule[4];
	   	nRF24_TXResult tx_res;
			uint8_t offFlag=0;
		  uint32_t curTime=0;
			
			
			uint8_t identNumber =distanceSensor;
			 
			 
			extern I2C_HandleTypeDef hi2c1;
			
			
		void SerialNumbFunc(){
		
		
			volatile uint32_t *UniqueID = (uint32_t *)0x1FFFF7E8;
	volatile uint32_t __UniqueID[3];
	__UniqueID[0] = UniqueID[0];
	__UniqueID[1] = UniqueID[1];
	__UniqueID[2] = UniqueID[2];

	SerialModule[0]=__UniqueID[2];
	SerialModule[1]=__UniqueID[2]>>8;
	SerialModule[2]=__UniqueID[2]>>16;
	SerialModule[3]=__UniqueID[2]>>24;
		
		
		}

		
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

	uint8_t Ident[]={identNumber,100,100,100,100,100,100,100,100,100};

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
		
		
			case 4:
		Ident[4]=tempArr[4];
		Ident[5]=tempArr[5];
			
		break;
			
	}

	
			nRF24_SetPowerMode(nRF24_PWR_DOWN);
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);
    nRF24_SetOperationalMode(nRF24_MODE_TX);
    nRF24_ClearIRQFlags();
	  nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR2);
    nRF24_SetPowerMode(nRF24_PWR_UP);	
	    
	    tx_res = nRF24_TransmitPacket(Ident, payload_length);// data load
	
			nRF24_SetPowerMode(nRF24_PWR_DOWN);
    nRF24_SetOperationalMode(nRF24_MODE_RX);
    nRF24_SetPowerMode(nRF24_PWR_UP);
    nRF24_CE_H();
}



void TransmitIdentification (){
uint8_t Ident[]={identNumber,100,100,100,100,100,100,100,100,100};


	Ident[1]=SerialModule[0];
	Ident[2]=SerialModule[1];
	Ident[3]=1;

	
	
  	nRF24_SetPowerMode(nRF24_PWR_DOWN);
    // Set TX power (maximum)
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);
    // Set operational mode (PTX == transmitter)
    nRF24_SetOperationalMode(nRF24_MODE_TX);
    // Clear any pending IRQ flags
    nRF24_ClearIRQFlags();
	nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR2);
    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);
	
	
	    	tx_res = nRF24_TransmitPacket(Ident, payload_length);
    	switch (tx_res) {
			case nRF24_TX_SUCCESS:
				
				break;
			case nRF24_TX_TIMEOUT:
				
				break;
			case nRF24_TX_MAXRT:
				
				break;
			default:
			
				break;
		}
    	
		
			nRF24_SetPowerMode(nRF24_PWR_DOWN);
		
		 // Set operational mode (PRX == receiver)
    nRF24_SetOperationalMode(nRF24_MODE_RX);

    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);

    // Put the transceiver to the RX mode
    nRF24_CE_H();

}





	
void OnAction(){

HAL_NVIC_DisableIRQ(EXTI0_IRQn); 
			offFlag=1;
	curTime=HAL_GetTick();	
if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == SET) 
{ 
__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	offFlag=1;
	curTime=HAL_GetTick();
	
} 
}

void NRF_tuning (){

     // RX/TX disabled
    nRF24_CE_L();

    while (!nRF24_Check()) {
 
			HAL_Delay(1000);
    }
		
		  // Initialize the nRF24L01 to its default state
    nRF24_Init();
		
		
    // Disable ShockBurst for all RX pipes
    nRF24_DisableAA(0xFF);

    // Set RF channel
    nRF24_SetRFChannel(95);

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



		nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 10); 

 
    // Set operational mode (PRX == receiver)
    nRF24_SetOperationalMode(nRF24_MODE_RX);
		
    nRF24_ClearIRQFlags();
	
    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);

    // Put the transceiver to the RX mode
    nRF24_CE_H();



}








