package NrfLib;


import static com.pi4j.wiringpi.Gpio.INPUT;
import static com.pi4j.wiringpi.Gpio.OUTPUT;
import static com.pi4j.wiringpi.Gpio.PUD_UP;
import static com.pi4j.wiringpi.Gpio.digitalRead;
import static com.pi4j.wiringpi.Gpio.digitalWrite;
import static com.pi4j.wiringpi.Gpio.pinMode;
import static com.pi4j.wiringpi.Gpio.pullUpDnControl;
import static com.pi4j.wiringpi.Gpio.wiringPiSetup;
import static com.pi4j.wiringpi.GpioUtil.DIRECTION_IN;
import static com.pi4j.wiringpi.GpioUtil.DIRECTION_OUT;
import static com.pi4j.wiringpi.GpioUtil.export;
import static com.pi4j.wiringpi.GpioUtil.unexport;
import static com.pi4j.wiringpi.Spi.wiringPiSPIDataRW;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import com.pi4j.wiringpi.Spi;

public class NrfLibraryClass  implements Registers{





    private static final int MISO = 21;
    /**
     * ce GPIO_11
     */
    private static final int CE = 13;
    /**
     * mosi GPIO_13
     */
    private static final int MOSI = 19;
    /**
     * sclk GPIO_14
     */
    private static final int SCLK = 23;
    /**
     * csn GPIO_0
     */
    private static final int CSN = 15;


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
    String nRF24_TEST_ADDR="nRF24";

    public void nRF24_RX_ON()   {
        nRF24_CE_H();

    }

    public void nRF24_RX_OFF()   {
        nRF24_CE_L();

    }




    public void  init()
    {

//        wiringPiSetup () ;
//        pinMode (2, OUTPUT) ;
//        pinMode (3, OUTPUT) ;


    }


   public byte nRF24_LL_RW(byte data) {

       byte temp_read;
       byte test=0;
       byte [] buf=new byte[1];
        buf[0]=data;

       Spi.wiringPiSPIDataRW(0, buf, 1);

       System.out.println("inp"+data);
       System.out.println( "out"+buf[0]);
        return buf[0];
    }


    public  byte nRF24_ReadReg(byte reg) {
        byte value;

        nRF24_CSN_L();
        nRF24_LL_RW((byte) (reg & nRF24_MASK_REG_MAP));
        value =nRF24_LL_RW(nRF24_CMD_NOP);
        nRF24_CSN_H();

        return value;
    }


    void nRF24_WriteReg(byte reg, byte value) {
        nRF24_CSN_L();
        if (reg < nRF24_CMD_W_REGISTER) {
            // This is a register access
            nRF24_LL_RW((byte) (nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP)));
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


//    void nRF24_ReadMBReg(byte reg, byte [] pBuf, byte count) {
//        byte temp_c=count;
//        nRF24_CSN_L();
//        nRF24_LL_RW(reg);
//        while (count>=0) {
//		pBuf[temp_c-count] = nRF24_LL_RW(nRF24_CMD_NOP);
//            count--;
//        }
//        nRF24_CSN_H();
//    }



    // Read a multi-byte register
// input:
//   reg - number of register to read
//   pBuf - pointer to the buffer for register data
//   count - number of bytes to read
    byte []  nRF24_ReadMBReg(byte reg, byte[] pBuf, byte count) {
        byte [] temp= new byte [count];
        byte temp_c=count;
        nRF24_CSN_L();
        nRF24_LL_RW(reg);
        while (count>0) {
            temp[temp_c-count] = nRF24_LL_RW(nRF24_CMD_NOP);
           count--;
        }
        nRF24_CSN_H();
        return temp;
    }

    // Write a multi-byte register
// input:
//   reg - number of register to write
//   pBuf - pointer to the buffer with data to write
//   count - number of bytes to write
    void nRF24_WriteMBReg(byte reg, byte []pBuf, byte count) {
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


    void nRF24_Init() {
        // Write to registers their initial values
        nRF24_WriteReg(nRF24_REG_CONFIG, (byte) 0x08);
        nRF24_WriteReg(nRF24_REG_EN_AA, (byte)0x3F);
        nRF24_WriteReg(nRF24_REG_EN_RXADDR, (byte)0x03);
        nRF24_WriteReg(nRF24_REG_SETUP_AW, (byte)0x03);
        nRF24_WriteReg(nRF24_REG_SETUP_RETR, (byte)0x03);
        nRF24_WriteReg(nRF24_REG_RF_CH, (byte)0x02);
        nRF24_WriteReg(nRF24_REG_RF_SETUP, (byte)0x0E);
        nRF24_WriteReg(nRF24_REG_STATUS, (byte)0x00);
        nRF24_WriteReg(nRF24_REG_RX_PW_P0, (byte)0x00);
        nRF24_WriteReg(nRF24_REG_RX_PW_P1, (byte)0x00);
        nRF24_WriteReg(nRF24_REG_RX_PW_P2, (byte)0x00);
        nRF24_WriteReg(nRF24_REG_RX_PW_P3, (byte)0x00);
        nRF24_WriteReg(nRF24_REG_RX_PW_P4, (byte)0x00);
        nRF24_WriteReg(nRF24_REG_RX_PW_P5, (byte)0x00);
        nRF24_WriteReg(nRF24_REG_DYNPD, (byte)0x00);
        nRF24_WriteReg(nRF24_REG_FEATURE, (byte)0x00);

        // Clear the FIFO's
       // nRF24_FlushRX();
      //  nRF24_FlushTX();

        // Clear any pending interrupt flags
      //  nRF24_ClearIRQFlags();

        // Deassert CSN pin (chip release)
        nRF24_CSN_H();
    }


    // Check if the nRF24L01 present
// return:
//   1 - nRF24L01 is online and responding
//   0 - received sequence differs from original
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
                System.out.print("|"+rxbuf[i]);
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
////////////////////////////////////////
private final int spiReadWrite(int spiData) {
    for (int i = 0; i < 8; i++) {
        if ((0x80 & spiData) == 0x80) digitalWrite(MOSI, 1);
        else digitalWrite(MOSI, 0);

        spiData <<= 1;
        digitalWrite(SCLK, 1);

        if (digitalRead(MISO)==1) spiData |= 0x01;
        digitalWrite(SCLK, 0);
    }
    return spiData & 0xff;
}

    private final int writeRegister(int regAddr, int writeData) {
        digitalWrite(CSN, 0);
        int val = spiReadWrite(regAddr);
        spiReadWrite(writeData);
        digitalWrite(CSN, 1);
        return val;
    }
    private final int readRegister(int regAddr) {
        digitalWrite(CSN, 0);
        spiReadWrite(regAddr);
        int val = spiReadWrite(0x00);
        digitalWrite(CSN, 1);
        return val;
    }


    public static void main(String[] args) {


//        wiringPiSetup () ;
//        pinMode (2, OUTPUT) ;
//        pinMode (3, OUTPUT) ;
//
        NrfLibraryClass test= new NrfLibraryClass();
//        System.out.println("nRF24L01+ check: ");
//     test.nRF24_CE_L();
//        while (test.nRF24_Check()!=1) {
//            System.out.println("FAIL\r\n");
//            try { Thread.sleep(1000);} catch (InterruptedException e){}
//        }
//


        int rxbuf=10;
        int ptr=4;
        byte i;
        String nRF24_TEST_ADDR="nRF24";


            System.out.println();
            // Write test TX address and read TX_ADDR register
            test.writeRegister((int)(nRF24_CMD_W_REGISTER | nRF24_REG_TX_ADDR), ptr);
            rxbuf=  test.readRegister((nRF24_CMD_R_REGISTER | nRF24_REG_TX_ADDR));
            // Compare buffers, return error on first mismatch

                if (rxbuf != ptr)
                    System.out.println(rxbuf);


            }





    }




