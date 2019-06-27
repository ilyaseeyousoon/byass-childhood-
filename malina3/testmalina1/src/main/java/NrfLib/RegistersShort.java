package NrfLib;

public interface RegistersShort {


// nRF24L0 instruction definitions
public static final short nRF24_CMD_R_REGISTER       =0x00; // Register read
    public static final short nRF24_CMD_W_REGISTER       =0x20;// Register write
    public static final short nRF24_CMD_R_RX_PAYLOAD     =0x61; // Read RX payload
    public static final short nRF24_CMD_W_TX_PAYLOAD     = 0xA0; // Write TX payload
    public static final short nRF24_CMD_FLUSH_TX         =  0xE1; // Flush TX FIFO
    public static final short nRF24_CMD_FLUSH_RX         =  0xE2; // Flush RX FIFO
    public static final short nRF24_CMD_REUSE_TX_PL      =  0xE3; // Reuse TX payload
    public static final short nRF24_CMD_LOCK_UNLOCK      =0x50; // Lock/unlock exclusive features
    public static final short nRF24_CMD_NOP              = 0xFF; // No operation (used for reading status register)

// nRF24L0 register definitions
public static final short nRF24_REG_CONFIG           =0x00; // Configuration register
    public static final short nRF24_REG_EN_AA            =0x01; // Enable "Auto acknowledgment"
    public static final short nRF24_REG_EN_RXADDR        =0x02; // Enable RX addresses
    public static final short nRF24_REG_SETUP_AW         =0x03; // Setup of address widths
    public static final short nRF24_REG_SETUP_RETR      =0x04; // Setup of automatic retransmit
    public static final short nRF24_REG_RF_CH            =0x05; // RF channel
    public static final short nRF24_REG_RF_SETUP         =0x06; // RF setup register
    public static final short nRF24_REG_STATUS          =0x07; // Status register
    public static final short nRF24_REG_OBSERVE_TX       =0x08; // Transmit observe register
    public static final short nRF24_REG_RPD              =0x09; // Received power detector
    public static final short nRF24_REG_RX_ADDR_P0       =0x0A; // Receive address data pipe 0
    public static final short nRF24_REG_RX_ADDR_P1       =0x0B; // Receive address data pipe 1
    public static final short nRF24_REG_RX_ADDR_P2       =0x0C; // Receive address data pipe 2
    public static final short nRF24_REG_RX_ADDR_P3       =0x0D; // Receive address data pipe 3
    public static final short nRF24_REG_RX_ADDR_P4       =0x0E; // Receive address data pipe 4
    public static final short nRF24_REG_RX_ADDR_P5       =0x0F; // Receive address data pipe 5
    public static final short nRF24_REG_TX_ADDR          =0x10; // Transmit address
    public static final short nRF24_REG_RX_PW_P0         =0x11; // Number of bytes in RX payload in data pipe 0
    public static final short nRF24_REG_RX_PW_P1        =0x12 ;// Number of bytes in RX payload in data pipe 1
    public static final short nRF24_REG_RX_PW_P2         =0x13; // Number of bytes in RX payload in data pipe 2
    public static final short nRF24_REG_RX_PW_P3        =0x14;// Number of bytes in RX payload in data pipe 3
    public static final short nRF24_REG_RX_PW_P4         =0x15; // Number of bytes in RX payload in data pipe 4
    public static final short nRF24_REG_RX_PW_P5        =0x16; // Number of bytes in RX payload in data pipe 5
    public static final short nRF24_REG_FIFO_STATUS      =0x17; // FIFO status register
    public static final short nRF24_REG_DYNPD           =0x1C; // Enable dynamic payload length
    public static final short nRF24_REG_FEATURE          =0x1D; // Feature register

// Register bits definitions
public static final short nRF24_CONFIG_PRIM_RX       =0x01; // PRIM_RX bit in CONFIG register
    public static final short nRF24_CONFIG_PWR_UP        =0x02; // PWR_UP bit in CONFIG register
    public static final short nRF24_FLAG_RX_DR           =0x40; // RX_DR bit (data ready RX FIFO interrupt)
    public static final short nRF24_FLAG_TX_DS           =0x20; // TX_DS bit (data sent TX FIFO interrupt)
    public static final short nRF24_FLAG_MAX_RT         =0x10;// MAX_RT bit (maximum number of TX retransmits interrupt)

// Register masks definitions
public static final short nRF24_MASK_REG_MAP         = 0x1F; // Mask bits[4:0] for CMD_RREG and CMD_WREG commands
    public static final short nRF24_MASK_CRC             =0x0C; // Mask for CRC bits [3:2] in CONFIG register
    public static final short nRF24_MASK_STATUS_IRQ     =0x70; // Mask for all IRQ bits in STATUS register
    public static final short nRF24_MASK_RF_PWR          =0x06; // Mask RF_PWR[2:1] bits in RF_SETUP register
    public static final short nRF24_MASK_RX_P_NO         =0x0E; // Mask RX_P_NO[3:1] bits in STATUS register
    public static final short nRF24_MASK_DATARATE        =0x28; // Mask RD_DR_[5,3] bits in RF_SETUP register
    public static final short nRF24_MASK_EN_RX           =0x3F; // Mask ERX_P[5:0] bits in EN_RXADDR register
    public static final short nRF24_MASK_RX_PW           =0x3F; // Mask [5:0] bits in RX_PW_Px register
    public static final short nRF24_MASK_RETR_ARD       = (byte) 0xF0; // Mask for ARD[7:4] bits in SETUP_RETR register
    public static final short nRF24_MASK_RETR_ARC        =0x0F; // Mask for ARC[3:0] bits in SETUP_RETR register
    public static final short nRF24_MASK_RXFIFO          =0x03; // Mask for RX FIFO status bits [1:0] in FIFO_STATUS register
    public static final short nRF24_MASK_TXFIFO         =0x30; // Mask for TX FIFO status bits [5:4] in FIFO_STATUS register
    public static final short nRF24_MASK_PLOS_CNT        = (byte) 0xF0; // Mask for PLOS_CNT[7:4] bits in OBSERVE_TX register
    public static final short nRF24_MASK_ARC_CNT         =0x0F;// Mask for ARC_CNT[3:0] bits in OBSERVE_TX register



    public static final short nRF24_ARD_NONE   =0x00;
    public static final short  nRF24_ARD_250us  = 0x00;
    public static final short   nRF24_ARD_500us  = 0x01;
    public static final short    nRF24_ARD_750us  = 0x02;
    public static final short     nRF24_ARD_1000us = 0x03;
    public static final short     nRF24_ARD_1250us = 0x04;
    public static final short     nRF24_ARD_1500us = 0x05;
    public static final short      nRF24_ARD_1750us = 0x06;
    public static final short    nRF24_ARD_2000us = 0x07;
    public static final short    nRF24_ARD_2250us = 0x08;
    public static final short    nRF24_ARD_2500us = 0x09;
    public static final short    nRF24_ARD_2750us =0x0A;
    public static final short    nRF24_ARD_3000us = 0x0B;
    public static final short    nRF24_ARD_3250us = 0x0C;
    public static final short    nRF24_ARD_3500us = 0x0D;
    public static final short    nRF24_ARD_3750us = 0x0E;
    public static final short    nRF24_ARD_4000us = 0x0F;



    // Data rate

    public static final short   nRF24_DR_250kbps = 0x20; // 250kbps data rate
    public static final short   nRF24_DR_1Mbps   = 0x00; // 1Mbps data rate
    public static final short   nRF24_DR_2Mbps   = 0x08;  // 2Mbps data rate


// RF output power in TX mode

    public static final short nRF24_TXPWR_18dBm = 0x00; // -18dBm
    public static final short nRF24_TXPWR_12dBm = 0x02; // -12dBm
    public static final short nRF24_TXPWR_6dBm  = 0x04; //  -6dBm
    public static final short nRF24_TXPWR_0dBm  = 0x06; //   0dBm


// CRC encoding scheme

    public static final short nRF24_CRC_off   = 0x00;// CRC disabled
    public static final short nRF24_CRC_1byte = 0x08; // 1-byte CRC
    public static final short nRF24_CRC_2byte = 0x0c;  // 2-byte CRC


// nRF24L01 power control

    public static final short nRF24_PWR_UP   = 0x02; // Power up
    public static final short nRF24_PWR_DOWN = 0x00;  // Power down


// Transceiver mode

    public static final short nRF24_MODE_RX = 0x01; // PRX
    public static final short nRF24_MODE_TX = 0x00; // PTX


// Enumeration of RX pipe addresses and TX address

    public static final short nRF24_PIPE0  = 0x00; // pipe0
    public static final short nRF24_PIPE1  =0x01; // pipe1
    public static final short nRF24_PIPE2  = 0x02; // pipe2
    public static final short nRF24_PIPE3  = 0x03; // pipe3
    public static final short nRF24_PIPE4  = 0x04; // pipe4
    public static final short nRF24_PIPE5  = 0x05; // pipe5
    public static final short nRF24_PIPETX = 0x06;  // TX address (not a pipe in fact)


// State of auto acknowledgment for specified pipe

    public static final short nRF24_AA_OFF = 0x00;
    public static final short nRF24_AA_ON  = 0x01;


// Status of the RX FIFO

    public static final short nRF24_STATUS_RXFIFO_DATA  = 0x00; // The RX FIFO contains data and available locations
    public static final short nRF24_STATUS_RXFIFO_EMPTY = 0x01; // The RX FIFO is empty
    public static final short nRF24_STATUS_RXFIFO_FULL  = 0x02; // The RX FIFO is full
    public static final short nRF24_STATUS_RXFIFO_ERROR = 0x03;  // Impossible state: RX FIFO cannot be empty and full at the same time


// Status of the TX FIFO

        public static final short nRF24_STATUS_TXFIFO_DATA  =0x00; // The TX FIFO contains data and available locations
        public static final short nRF24_STATUS_TXFIFO_EMPTY =0x01; // The TX FIFO is empty
        public static final short nRF24_STATUS_TXFIFO_FULL  = 0x02; // The TX FIFO is full
        public static final short nRF24_STATUS_TXFIFO_ERROR = 0x03;  // Impossible state: TX FIFO cannot be empty and full at the same time


    // Addresses of the RX_PW_P# registers
    public static final short[] nRF24_RX_PW_PIPE = {
        nRF24_REG_RX_PW_P0,
                nRF24_REG_RX_PW_P1,
                nRF24_REG_RX_PW_P2,
                nRF24_REG_RX_PW_P3,
                nRF24_REG_RX_PW_P4,
                nRF24_REG_RX_PW_P5
    };

    // Addresses of the address registers
    public static final short [] nRF24_ADDR_REGS = {
        nRF24_REG_RX_ADDR_P0,
                nRF24_REG_RX_ADDR_P1,
                nRF24_REG_RX_ADDR_P2,
                nRF24_REG_RX_ADDR_P3,
                nRF24_REG_RX_ADDR_P4,
                nRF24_REG_RX_ADDR_P5,
                nRF24_REG_TX_ADDR
    };



    public static final short nRF24_RX_PIPE0  = 0x00; // Packet received from the PIPE#0
    public static final short nRF24_RX_PIPE1  = 0x01; // Packet received from the PIPE#1
    public static final short nRF24_RX_PIPE2  = 0x02; // Packet received from the PIPE#2
    public static final short nRF24_RX_PIPE3  = 0x03; // Packet received from the PIPE#3
    public static final short nRF24_RX_PIPE4  = 0x04; // Packet received from the PIPE#4
    public static final short nRF24_RX_PIPE5  = 0x05; // Packet received from the PIPE#5
    public static final short nRF24_RX_EMPTY  =  0xff;  // The RX FIFO is empty













}
