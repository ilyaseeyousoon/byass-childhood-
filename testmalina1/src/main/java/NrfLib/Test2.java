package NrfLib;

import com.pi4j.wiringpi.Spi;


import static com.pi4j.wiringpi.Gpio.*;
import com.pi4j.wiringpi.Spi;
import com.pi4j.io.spi.SpiChannel;
import com.pi4j.io.spi.SpiDevice;
import com.pi4j.io.spi.SpiFactory;
import jpigpio.PigpioException;
import rf24j.Util;

import java.io.IOException;

public class Test2 implements Registers {

    static byte[] nRF24_payload_test = new byte[6];


    String nRF24_TEST_ADDR="nRF24";
    public static SpiDevice spi = null;

    static  byte[] nRF24_ADDR0 = {(byte) 0x01,(byte)0x01,(byte)0xE7, (byte)0x1C, (byte)0xE1};
    static byte[] nRF24_ADDR1 ={(byte) 0x01,(byte)0x01,(byte)0xE7, (byte)0x1C, (byte)0xE2};
    static  byte[] nRF24_ADDR2 ={ (byte) 0xE3 };
    static  byte[] nRF24_ADDR3 ={ (byte) 0xE4 };
    static  byte[] nRF24_ADDR4 ={ (byte) 0xE5 };
    static  byte[] nRF24_ADDR5 ={(byte)  0xE6 };

//    static const uint8_t nRF24_ADDR2[] = {  0x01,0x01,0xE7, 0x1C, 0xE3 };
//    static const uint8_t nRF24_ADDR3[] = {  0x01,0x01,0xE7, 0x1C, 0xE4 };
//    static const uint8_t nRF24_ADDR4[] = {  0x01,0x01,0xE7, 0x1C, 0xE5 };
//    static const uint8_t nRF24_ADDR5[] = {  0x01,0x01,0xE7, 0x1C, 0xE6 };
//
//    static  byte[] nRF24_ADDR2 ={(byte) 0xE3,(byte)0x01,(byte)0xE7, (byte)0x1C, (byte)0xE3};
//    static  byte[] nRF24_ADDR3={(byte) 0xE4,(byte)0x01,(byte)0xE7, (byte)0x1C, (byte)0xE4};
//    static  byte[] nRF24_ADDR4 ={(byte) 0xE5,(byte)0x01,(byte)0xE7, (byte)0x1C, (byte)0xE5};
//    static  byte[] nRF24_ADDR5 ={(byte) 0xE6,(byte)0x01,(byte)0xE7, (byte)0x1C, (byte)0xE6};


    public  Test2()throws IOException{
        Spi.wiringPiSPISetupMode(0,800000,0);

//       spi = SpiFactory.getInstance(SpiChannel.CS0,
//               8000000, // default spi speed 1 MHz
//               SpiDevice.DEFAULT_SPI_MODE); // default spi mode 0


    }
    // CE (chip enable) pin (PB11)
    public void nRF24_CE_L()   {
        digitalWrite(2, 0);

    }

    public void nRF24_CE_H()   {
        digitalWrite(2, 1);

    }


// CSN (chip select negative) pin (PA4)


    // CE (chip enable) pin (PB11)
    public void nRF24_CSN_L()   {
        digitalWrite(3, 0);

    }

    public void nRF24_CSN_H()   {
        digitalWrite(3, 1);

    }



    void nRF24_WriteMBReg(byte reg, byte []pBuf, byte count) throws IOException{
        byte temp_c=count;
        nRF24_CSN_L();
        nRF24_LL_RW(reg);
        while (count>0) {
            nRF24_LL_RW(pBuf[temp_c-count]);
//            System.out.print("|"+pBuf[temp_c-count]);
//            System.out.println();
            count--;
        }
        nRF24_CSN_H();
    }


    byte []  nRF24_ReadMBReg(byte reg, byte[] pBuf, byte count)throws IOException {
        byte [] temp= new byte [count];
        byte temp_c=count;
        nRF24_CSN_L();
        nRF24_LL_RW(reg);
        while (count!=0) {
            temp[temp_c-count] = nRF24_LL_RW(nRF24_CMD_NOP);
            count--;
        }
        nRF24_CSN_H();
        return temp;
    }


    public byte nRF24_LL_RW(byte data) throws IOException{

        byte temp_read;
        byte test=0;
        byte [] buf=new byte[1];
        buf[0]=data;


       // System.out.print("return!");

        //System.out.println(spi.write(data));
        //System.out.println(Spi.wiringPiSPIDataRW(0,buf, 1));
        Spi.wiringPiSPIDataRW(0, buf, 1);
        //buf= spi.write(data);

        //System.out.println("inp"+(byte)data);
       // System.out.println( "out"+(byte)buf[0]);
        return buf[0];
    }






    public byte nRF24_Check() {
        byte[] rxbuf=new byte[5];
        byte[] ptr;
        byte i;
        try {
            ptr =nRF24_TEST_ADDR.getBytes("ASCII");
            for (byte b : ptr ) {
                System.out.print("|"+b);
            }
            System.out.println();
            // Write test TX address and read TX_ADDR register
            nRF24_WriteMBReg((byte)(nRF24_CMD_W_REGISTER | nRF24_REG_TX_ADDR), ptr, (byte)5);
            rxbuf=nRF24_ReadMBReg((byte)(nRF24_CMD_R_REGISTER | nRF24_REG_TX_ADDR), rxbuf, (byte)5);
            // Compare buffers, return error on first mismatch
            for (i = 0; i < 5; i++) {
               // System.out.print("|"+rxbuf[i]);
                if (rxbuf[i] != ptr[i]) {
                    System.out.println();
                    return 0;
                }
            }
        }catch (Exception e){
            System.out.println(e);
        }
        return 1;
    }




    byte nRF24_ReadReg(byte reg) throws IOException {
        byte value;

        nRF24_CSN_L();
        nRF24_LL_RW((byte) (reg & nRF24_MASK_REG_MAP));
        value = nRF24_LL_RW(nRF24_CMD_NOP);
        nRF24_CSN_H();

        return value;
    }



    // Get value of the STATUS register
// return: value of STATUS register
    byte nRF24_GetStatus() throws IOException {
        return nRF24_ReadReg(nRF24_REG_STATUS);
    }

    // Get pending IRQ flags
// return: current status of RX_DR, TX_DS and MAX_RT bits of the STATUS register
    byte nRF24_GetIRQFlags() throws IOException {
        return (byte) (nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_STATUS_IRQ);
    }

    // Get status of the RX FIFO
// return: one of the nRF24_STATUS_RXFIFO_xx values
    byte nRF24_GetStatus_RXFIFO() throws IOException {
        return (byte) (nRF24_ReadReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_RXFIFO);
    }

    // Get status of the TX FIFO
// return: one of the nRF24_STATUS_TXFIFO_xx values
// note: the TX_REUSE bit ignored
    byte nRF24_GetStatus_TXFIFO() throws IOException {
        return (byte) ((nRF24_ReadReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_TXFIFO) >> 4);
    }

    // Get pipe number for the payload available for reading from RX FIFO
// return: pipe number or 0x07 if the RX FIFO is empty
    byte nRF24_GetRXSource() throws IOException {
        return (byte) ((nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1);
    }

    // Get auto retransmit statistic
// return: value of OBSERVE_TX register which contains two counters encoded in nibbles:
//   high - lost packets count (max value 15, can be reseted by write to RF_CH register)
//   low  - retransmitted packets count (max value 15, reseted when new transmission starts)
    byte nRF24_GetRetransmitCounters() throws IOException {
        return (nRF24_ReadReg(nRF24_REG_OBSERVE_TX));
    }

    // Reset packet lost counter (PLOS_CNT bits in OBSERVER_TX register)
    void nRF24_ResetPLOS() throws IOException{
        byte reg;

        // The PLOS counter is reset after write to RF_CH register
        reg = nRF24_ReadReg(nRF24_REG_RF_CH);
        nRF24_WriteReg(nRF24_REG_RF_CH, reg);
    }

    // Flush the TX FIFO
    void nRF24_FlushTX()throws IOException {
        nRF24_WriteReg(nRF24_CMD_FLUSH_TX, nRF24_CMD_NOP);
    }

    // Flush the RX FIFO
    void nRF24_FlushRX()throws IOException {
        nRF24_WriteReg(nRF24_CMD_FLUSH_RX, nRF24_CMD_NOP);
    }

    // Clear any pending IRQ flags
    void nRF24_ClearIRQFlags() throws IOException{
        byte reg;

        // Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
        reg  = nRF24_ReadReg(nRF24_REG_STATUS);
        reg |= nRF24_MASK_STATUS_IRQ;
        nRF24_WriteReg(nRF24_REG_STATUS, reg);
    }

    // Write TX payload
// input:
//   pBuf - pointer to the buffer with payload data
//   length - payload length in bytes
    void nRF24_WritePayload(byte [] pBuf, byte length) throws IOException {
        nRF24_WriteMBReg(nRF24_CMD_W_TX_PAYLOAD, pBuf, length);
    }

    // Write a new value to register
// input:
//   reg - number of register to write
//   value - value to write
    void nRF24_WriteReg(byte reg, byte value)throws IOException {
        nRF24_CSN_L();
        if ((reg&0xFF) < nRF24_CMD_W_REGISTER) {
            // This is a register access
            nRF24_LL_RW((byte)(nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP)));
            nRF24_LL_RW(value);
        } else {
            // This is a single byte command or future command/register
            nRF24_LL_RW(reg);
            if ((reg != nRF24_CMD_FLUSH_TX) && (reg != nRF24_CMD_FLUSH_RX) &&
            (reg != nRF24_CMD_REUSE_TX_PL) && (reg != nRF24_CMD_NOP)) {
                // Send register value
                nRF24_LL_RW(value);
            }
        }
        nRF24_CSN_H();
    }

    void nRF24_DisableAA(byte pipe) throws IOException {
        byte reg=0;
       // System.out.println("pipe!="+(pipe&0xFF));
        if ((pipe&0xFF) > 5) {
            // Disable Auto-ACK for ALL pipes
            nRF24_WriteReg(nRF24_REG_EN_AA, (byte) 0x00);
        } else {
            // Clear bit in the EN_AA register
            reg  = nRF24_ReadReg(nRF24_REG_EN_AA);
            reg &= ~(1 << pipe);
            nRF24_WriteReg(nRF24_REG_EN_AA, reg);
        }
    }



    void nRF24_Init()throws IOException  {
        // Write to registers their initial values
        nRF24_WriteReg((byte) nRF24_REG_CONFIG, (byte)0x08);
        nRF24_WriteReg((byte)nRF24_REG_EN_AA, (byte)0x3F);
        nRF24_WriteReg((byte)nRF24_REG_EN_RXADDR, (byte)0x03);
        nRF24_WriteReg((byte)nRF24_REG_SETUP_AW, (byte)0x03);
        nRF24_WriteReg((byte)nRF24_REG_SETUP_RETR, (byte)0x03);
        nRF24_WriteReg((byte)nRF24_REG_RF_CH, (byte)0x02);
        nRF24_WriteReg((byte)nRF24_REG_RF_SETUP, (byte)0x0E);
        nRF24_WriteReg((byte)nRF24_REG_STATUS, (byte)0x00);
        nRF24_WriteReg((byte)nRF24_REG_RX_PW_P0, (byte)0x00);
        nRF24_WriteReg((byte)nRF24_REG_RX_PW_P1, (byte)0x00);
        nRF24_WriteReg((byte)nRF24_REG_RX_PW_P2, (byte)0x00);
        nRF24_WriteReg((byte)nRF24_REG_RX_PW_P3, (byte)0x00);
        nRF24_WriteReg((byte)nRF24_REG_RX_PW_P4, (byte)0x00);
        nRF24_WriteReg((byte)nRF24_REG_RX_PW_P5, (byte)0x00);
        nRF24_WriteReg((byte)nRF24_REG_DYNPD, (byte)0x00);
        nRF24_WriteReg((byte)nRF24_REG_FEATURE, (byte)0x00);

        // Clear the FIFO's
        nRF24_FlushRX();
        nRF24_FlushTX();

        // Clear any pending interrupt flags
        nRF24_ClearIRQFlags();

        // Deassert CSN pin (chip release)
        nRF24_CSN_H();
    }


    void nRF24_SetRFChannel(byte channel) throws IOException {
        nRF24_WriteReg(nRF24_REG_RF_CH, channel);
      //  System.out.println(channel );
    }

    // Set automatic retransmission parameters
// input:
//   ard - auto retransmit delay, one of nRF24_ARD_xx values
//   arc - count of auto retransmits, value form 0 to 15
// note: zero arc value means that the automatic retransmission disabled
    void nRF24_SetAutoRetr(byte ard, byte arc) throws IOException {
        // Set auto retransmit settings (SETUP_RETR register)
        nRF24_WriteReg(nRF24_REG_SETUP_RETR, (byte)((ard << 4) | (arc & nRF24_MASK_RETR_ARC)));
    }

    // Set of address widths
// input:
//   addr_width - RX/TX address field width, value from 3 to 5
// note: this setting is common for all pipes
    void nRF24_SetAddrWidth(byte addr_width) throws IOException {
        nRF24_WriteReg(nRF24_REG_SETUP_AW, (byte) (addr_width - 2));
    }

    // Set static RX address for a specified pipe
// input:
//   pipe - pipe to configure address, one of nRF24_PIPEx values
//   addr - pointer to the buffer with address
// note: pipe can be a number from 0 to 5 (RX pipes) and 6 (TX pipe)
// note: buffer length must be equal to current address width of transceiver
// note: for pipes[2..5] only first byte of address will be written because
//       other bytes of address equals to pipe1
// note: for pipes[2..5] only first byte of address will be written because
//       pipes 1-5 share the four most significant address bytes
    void nRF24_SetAddr(byte pipe,  byte[] addr) throws IOException {
        byte addr_width;

        // RX_ADDR_Px register
        switch (pipe) {
            case nRF24_PIPETX:
            case nRF24_PIPE0:
            case nRF24_PIPE1:
                // Get address width
                addr_width = (byte) (nRF24_ReadReg(nRF24_REG_SETUP_AW) + 1);
                // Write address in reverse order (LSByte first)

                byte temp_c=  (addr_width);
                nRF24_CSN_L();
               // System.out.println("nRF24_CSN_L();" );
                nRF24_LL_RW((byte) (nRF24_CMD_W_REGISTER | nRF24_ADDR_REGS[pipe]));

//                for (int i = 0; i < 5; i++) {
//                     System.out.print("|"+(addr[i]&0xFF));
//                    System.out.println();
//                }

                while (addr_width>=0) {
//                    System.out.print("before "+addr_width+" "+(addr[addr_width]&0xFF));
//                    System.out.println();
                     nRF24_LL_RW( addr[addr_width]);
        //            System.out.println(addr[addr_width]  );
//                    System.out.print("after "+addr_width+" "+(addr[addr_width]&0xFF));
//                    System.out.println();
                    addr_width--;
                }





                nRF24_CSN_H();
                break;
            case nRF24_PIPE2:
            case nRF24_PIPE3:
            case nRF24_PIPE4:
            case nRF24_PIPE5:
                // Write address LSBbyte (only first byte from the addr buffer)
                nRF24_WriteReg(nRF24_ADDR_REGS[pipe], addr[0]);
                break;
            default:
                // Incorrect pipe number -> do nothing
                break;
        }
    }

    void nRF24_SetDataRate(byte data_rate) throws IOException {
        byte reg;

        // Configure RF_DR_LOW[5] and RF_DR_HIGH[3] bits of the RF_SETUP register
        reg  = nRF24_ReadReg(nRF24_REG_RF_SETUP);
        //System.out.println(reg+"reg");
        reg &= ~nRF24_MASK_DATARATE;

        reg |= data_rate;

        nRF24_WriteReg(nRF24_REG_RF_SETUP, reg);
    }

    // Configure a specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
//   aa_state - state of auto acknowledgment, one of nRF24_AA_xx values
//   payload_len - payload length in bytes
    void nRF24_SetRXPipe(byte pipe, byte aa_state, byte payload_len) throws IOException {
        byte reg;

        // Enable the specified pipe (EN_RXADDR register)
        reg = (byte) ((nRF24_ReadReg(nRF24_REG_EN_RXADDR) | (1 << pipe)) & nRF24_MASK_EN_RX);
        nRF24_WriteReg(nRF24_REG_EN_RXADDR, reg);
      //  System.out.println("test "+reg);
        // Set RX payload length (RX_PW_Px register)
        nRF24_WriteReg(nRF24_RX_PW_PIPE[pipe], (byte) (payload_len & nRF24_MASK_RX_PW));

        // Set auto acknowledgment for a specified pipe (EN_AA register)
        reg = nRF24_ReadReg(nRF24_REG_EN_AA);
        if (aa_state == nRF24_AA_ON) {
            reg |=  (1 << pipe);
        } else {
            reg &= ~(1 << pipe);

        }
        nRF24_WriteReg(nRF24_REG_EN_AA, (byte)reg);
    }

    // Disable specified RX pipe
// input:
//   PIPE - number of RX pipe, value from 0 to 5
    void nRF24_ClosePipe(byte pipe) throws IOException {
        byte reg;

        reg  = nRF24_ReadReg(nRF24_REG_EN_RXADDR);
        reg &= ~(1 << pipe);
        reg &= nRF24_MASK_EN_RX;
        nRF24_WriteReg(nRF24_REG_EN_RXADDR, reg);
    }

    // Enable the auto retransmit (a.k.a. enhanced ShockBurst) for the specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
    void nRF24_EnableAA(byte pipe) throws IOException {
        byte reg;

        // Set bit in EN_AA register
        reg  = nRF24_ReadReg(nRF24_REG_EN_AA);

        reg |= (1 << pipe);
        nRF24_WriteReg(nRF24_REG_EN_AA, reg);
    }


    void nRF24_SetCRCScheme(byte scheme) throws IOException {
        byte reg;

        // Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register
        reg  = nRF24_ReadReg(nRF24_REG_CONFIG);
        reg &= ~nRF24_MASK_CRC;
        reg |= (scheme & nRF24_MASK_CRC);
        nRF24_WriteReg(nRF24_REG_CONFIG, reg);
    }

    void nRF24_SetTXPower(byte tx_pwr) throws IOException {
        byte reg;

        // Configure RF_PWR[2:1] bits of the RF_SETUP register
        reg  = nRF24_ReadReg(nRF24_REG_RF_SETUP);
        reg &= ~nRF24_MASK_RF_PWR;
        reg |= tx_pwr;
        nRF24_WriteReg(nRF24_REG_RF_SETUP, reg);
    }


    void nRF24_SetOperationalMode(byte mode) throws IOException {
        byte reg;

        // Configure PRIM_RX bit of the CONFIG register
        reg  = nRF24_ReadReg(nRF24_REG_CONFIG);
        reg &= ~nRF24_CONFIG_PRIM_RX;
        reg |= (byte)(mode & nRF24_CONFIG_PRIM_RX);
        nRF24_WriteReg(nRF24_REG_CONFIG, reg);
    }


    void nRF24_SetPowerMode(byte mode) throws IOException {
        byte reg;

        reg = nRF24_ReadReg(nRF24_REG_CONFIG);
        if (mode == nRF24_PWR_UP) {
            // Set the PWR_UP bit of CONFIG register to wake the transceiver
            // It goes into Stanby-I mode with consumption about 26uA
            reg |= nRF24_CONFIG_PWR_UP;
        } else {
            // Clear the PWR_UP bit of CONFIG register to put the transceiver
            // into power down mode with consumption about 900nA
            reg &= ~nRF24_CONFIG_PWR_UP;
        }
        nRF24_WriteReg(nRF24_REG_CONFIG, reg);
    }


    byte nRF24_ReadPayload(byte [] pBuf, byte length) throws IOException {
        byte pipe;

        // Extract a payload pipe number from the STATUS register
        pipe = (byte) ((nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1);
        //System.out.println("pipe= "+pipe);
        // RX FIFO empty?
        if (pipe < 6) {
            // Get payload length
		length = nRF24_ReadReg(nRF24_RX_PW_PIPE[pipe]);
           // System.out.println("length "+length);
            // Read a payload from the RX FIFO


            this.nRF24_payload_test=nRF24_ReadMBReg(nRF24_CMD_R_RX_PAYLOAD, pBuf, length);


            return pipe;
        }

        // The RX FIFO is empty
	length = 0;

        return nRF24_RX_EMPTY;
    }








    public String printDetails()throws IOException {
        String p = "";

        byte pwReg[] = {0,0,0,0,0,0};
        byte txAddr[] = new byte[5];
        pwReg= nRF24_ReadMBReg((byte)0x10, pwReg, (byte)5);
        p += "\nTX_ADDR         = 0x"+ Util.bytesToHex(Util.reverseArray(txAddr));
            nRF24_WriteMBReg((byte)(0x11), pwReg, (byte)5);
            pwReg= nRF24_ReadMBReg((byte)0x11, pwReg, (byte)5);
        p += "\nRX_PW_P0-6      = ";
        for(byte b:pwReg)
        p += "0x"+String.format("%02x",b)+"  ";
        p += "\n";
        byte rfChannel = nRF24_ReadReg((byte) 0x05);
        p += "\nRF_CH           = 0x"+ String.format("%02x",rfChannel);
        byte w = nRF24_ReadReg((byte) 0x03);
        p += "\nWIDTH           = 0x"+ String.format("%02x",w);

        pwReg= nRF24_ReadMBReg((byte)nRF24_REG_RX_ADDR_P0, pwReg, (byte)5);
        // addresses are stored LSB first, so we need to reverse it in order to print it correctly
        p += "\nRX_ADDR_P0-1    = 0x"+ Util.bytesToHex(Util.reverseArray(pwReg));
        pwReg= nRF24_ReadMBReg((byte)nRF24_REG_RX_ADDR_P1, pwReg, (byte)5);
        // addresses are stored LSB first, so we need to reverse it in order to print it correctly
        p += "\nRX_ADDR_P0-2    = 0x"+ Util.bytesToHex(Util.reverseArray(pwReg));
        pwReg= nRF24_ReadMBReg((byte)nRF24_REG_RX_ADDR_P2, pwReg, (byte)5);
        // addresses are stored LSB first, so we need to reverse it in order to print it correctly
        p += "\nRX_ADDR_P0-3    = 0x"+ Util.bytesToHex(Util.reverseArray(pwReg));
        pwReg= nRF24_ReadMBReg((byte)nRF24_REG_RX_ADDR_P3, pwReg, (byte)5);
        // addresses are stored LSB first, so we need to reverse it in order to print it correctly
        p += "\nRX_ADDR_P0-4    = 0x"+ Util.bytesToHex(Util.reverseArray(pwReg));
        pwReg= nRF24_ReadMBReg((byte)nRF24_REG_RX_ADDR_P4, pwReg, (byte)5);

        // addresses are stored LSB first, so we need to reverse it in order to print it correctly
        p += "\nRX_ADDR_P0-5    = 0x"+ Util.bytesToHex(Util.reverseArray(pwReg));


        byte setupReg = nRF24_ReadReg((byte) 0x06);
        p += "\nRF_SETUP        = 0x"+ String.format("%02x",setupReg) + "  " + String.format("%8s", Integer.toBinaryString(setupReg & 0xFF)).replace(' ', '0');


        byte dataRate = 0;
        if ((setupReg & 1<<5) > 0) dataRate += 2;
        if ((setupReg & 1<<3) > 0) dataRate += 1;
String  temp="";
        switch (dataRate) {
            case 0b00:
                temp+= "1 Mbps";
                break;
            case 0b01:
                temp+= "2 Mbps";
                break;
            case 0b10:
                temp+= "256 kbps";
                break;
            case 0x11:
                temp+= "reserved";
                break;
        }

        byte configReg = nRF24_ReadReg((byte) 0x00);
        p += "\nCONFIG          = 0x"+ String.format("%02x",configReg) + "  " + String.format("%8s", Integer.toBinaryString(configReg & 0xFF)).replace(' ', '0');


        p += "\nData Rate       = " +temp;
        p += "\nMODEL           = ???";
        p += "\nCRC Length      = ";

        if ((configReg & (1<<2)) == (1<<2) )
            p += "16 bits";
        else
            p += "8 bits";





        p += "\nRF Channel      = " + rfChannel;

        byte enRX = nRF24_ReadReg((byte) 0x02);
        p += "\nEN_RXADDR       = 0x"+ String.format("%02x",enRX) + "  " + String.format("%8s", Integer.toBinaryString(enRX & 0xFF)).replace(' ', '0');

        byte enAA = nRF24_ReadReg((byte) 0x01);
        p += "\nEN_AA           = 0x"+ String.format("%02x",enAA) + "  " + String.format("%8s", Integer.toBinaryString(enAA & 0xFF)).replace(' ', '0');

        byte i = nRF24_ReadReg(nRF24_REG_SETUP_AW);
        byte aw = (byte) ((i & 0x03) + 2);
        // RX_ADDR_P0
        pwReg=nRF24_ReadMBReg(nRF24_REG_RX_ADDR_P1,pwReg, (byte) 5);

        p += "\nPipe0addr          = 0x"+ Util.bytesToHex(pwReg);




        return p;
    }

    public static void main(String[] args)throws IOException {

         byte[] nRF24_payload = new byte[6];

        byte pipe = 99;

        byte payload_length = 0;

        wiringPiSetup();
        pinMode(2, OUTPUT);
        pinMode(3, OUTPUT);

        Test2 test = new Test2();
        System.out.println("nRF24L01+ check: ");
        test.nRF24_CE_L();
        while (test.nRF24_Check() != 1) {
            System.out.println("FAIL\r\n");
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
            }
        }
        System.out.println("nRF24L01+ check: OK ");
        test.nRF24_Init();


        test.nRF24_DisableAA((byte) 0xFF);

        // Set RF channel
        test.nRF24_SetRFChannel((byte) 115);

        // Set data rate
        test.nRF24_SetDataRate(nRF24_DR_250kbps);

        // Set CRC scheme
        test.nRF24_SetCRCScheme(nRF24_CRC_2byte);

        // Set address width, its common for all pipes (RX and TX)
        test.nRF24_SetAddrWidth((byte) 5);


        test.nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR0); // program address for RX pipe #0
        test.nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR1); // program address for RX pipe #1
        test.nRF24_SetAddr(nRF24_PIPE2, nRF24_ADDR2); // program address for RX pipe #2
       test.nRF24_SetAddr(nRF24_PIPE3, nRF24_ADDR3); // program address for RX pipe #3
        test.nRF24_SetAddr(nRF24_PIPE4, nRF24_ADDR4); // program address for RX pipe #4
        test.nRF24_SetAddr(nRF24_PIPE5, nRF24_ADDR5); // program address for RX pipe #5
        test.nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, (byte) 6);
        test.nRF24_SetRXPipe(nRF24_PIPE0, nRF24_AA_OFF, (byte) 6);
       test.nRF24_SetRXPipe(nRF24_PIPE2, nRF24_AA_OFF, (byte) 6);
        test.nRF24_SetRXPipe(nRF24_PIPE3, nRF24_AA_OFF, (byte) 6);
        test.nRF24_SetRXPipe(nRF24_PIPE4, nRF24_AA_OFF, (byte) 6);
        test.nRF24_SetRXPipe(nRF24_PIPE5, nRF24_AA_OFF, (byte) 6);



        // Set operational mode (PRX == receiver)
        test.nRF24_SetOperationalMode(nRF24_MODE_RX);

        // Wake the transceiver
        test.nRF24_SetPowerMode(nRF24_PWR_UP);

        System.out.println(test. printDetails());
        // Put the transceiver to the RX mode
        test.nRF24_CE_H();
        payload_length=6;
        long oldMs=0;
        long resultMs=0;
        long countMs=0;

        int X_temp = 0;
        int Y_temp = 0;

while(1==1) {
    if (test.nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
        // Get a payload from the transceiver

        pipe = test.nRF24_ReadPayload(nRF24_payload, payload_length);

//        System.out.println("pipe= " + pipe);
//        // Clear all pending IRQ flags
//        test.nRF24_ClearIRQFlags();
//
//        System.out.print("seconds = 0.");
//        System.out.println((System.currentTimeMillis()-oldMs)/100
//        );
        oldMs= System.currentTimeMillis();
        countMs++;
//        System.out.print("countMs = ");
//        System.out.println(countMs);
//        System.out.print("nRF24_payload_test = ");
//        System.out.println(Util.bytesToHex(nRF24_payload_test));

        X_temp = (int) nRF24_payload_test[2] * 256 + (int) nRF24_payload_test[3];
        Y_temp = (int) nRF24_payload_test[4] * 256 + (int) nRF24_payload_test[5];
        System.out.println("X_temp="+X_temp);
        System.out.println("Y_temp="+Y_temp);

    }
    try {
        Thread.sleep(100);
    } catch (InterruptedException e) {
    }
    //System.out.println("WAIT");
}
    }
}







