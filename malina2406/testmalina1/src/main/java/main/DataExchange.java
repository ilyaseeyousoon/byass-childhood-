package main;


import jpigpio.*;
import rf24j.RF24;
import jpigpio.Pigpio;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.lang.*;

public class DataExchange {

    private RF24 rf24;
    private RF24 rf25;

    int cePin = 27;  //GPIO number, e.g. GPIO 22 = PIN 15
    int csnPin = 22;  //GPIO number, e.g. GPIO 8 = PIN 24
    int cePin25 = 5;  //GPIO number, e.g. GPIO 22 = PIN 15
    int csnPin25 = 6;  //GPIO number, e.g. GPIO 8 = PIN 24

    byte role = 0;
    String cmd = "R";  // start with sender role

    byte counter = 0;
    byte senderData[] = new byte[]{1, 2, 3, 4};
    byte receiverReply[] = new byte[]{99, 99, 0, 0, 0, 0, 0};
    byte data32[] = new byte[6];
    byte datatemp[] = new byte[6];
    long timeout = 0;

    // set remote device address - to which data will be sent and from which data will be received
    byte rcvAddr[] = {(byte) 0xE1, (byte) 0x1C, (byte) 0xE7, (byte) 0x01, (byte) 0x01};
    // set transmitter device address - from which data will be sent
    byte sndAddr[] = {(byte) 0xE2, (byte) 0x1C, (byte) 0xE7, (byte) 0x01, (byte) 0x01};

    int X_temp = 0;
    int Y_temp = 0;

    long oldMs=0;
    long resultMs=0;
    long countMs=0;

    JPigpio pigpio = new PigpioSocket("localhost", 8888);





    public DataExchange() throws PigpioException {


           // JPigpio pigpio = new Pigpio();


            System.out.println("Creating pigpio...");

            System.out.println("Going to initialize pigpio...");
            pigpio.gpioInitialize();
            System.out.println("Creating RF24...");



        rf25 = new RF24(pigpio);
        if (!rf25.init(cePin, csnPin,0)) {
            p("Failed to initialize nRF module. Module not present?");
            rf25.terminate();
            pigpio.gpioTerminate();
            return;
        }

        rf25.setAddressWidth(5);
        // following params should be configured the same as the other side
        rf25.setPayloadSize(6);                // 32 bytes payload
        rf25.setChannel(115);                    // RF channel
        //rf24.setRetries(5,15);   				// 5 retries, 15x250ms delay between retries
        rf25.setCRCLength(2);                    // 16 bit CRC
        rf25.setDataRate(RF24.RF24_250KBPS);    // 1Mbit/s data rate
        rf25.setAutoACK(false);                    // expecting automatic acknowledgements from receiver
        rf25.setPALevel(RF24.RF24_PA_LOW);  // low power - testing devices won't be so far apart
        rf25.clearRegisterBits(RF24.CONFIG_REGISTER, (byte) (1 << RF24.MASK_RX_DR));
        System.out.println(rf25.printDetails());





        rf24 = new RF24(pigpio);

        p("Initializing...");
        if (!rf24.init(cePin25, csnPin25,0)) {
            p("Failed to initialize nRF module. Module not present?");
            rf24.terminate();
            pigpio.gpioTerminate();
            return;
        }


        rf24.setAddressWidth(5);
        // following params should be configured the same as the other side
        rf24.setPayloadSize(6);                // 32 bytes payload
        rf24.setChannel(115);                    // RF channel
        //rf24.setRetries(5,15);   				// 5 retries, 15x250ms delay between retries
        rf24.setCRCLength(2);                    // 16 bit CRC
        rf24.setDataRate(RF24.RF24_250KBPS);    // 1Mbit/s data rate
        rf24.setAutoACK(false);                    // expecting automatic acknowledgements from receiver
        rf24.setPALevel(RF24.RF24_PA_LOW);  // low power - testing devices won't be so far apart
        rf24.clearRegisterBits(RF24.CONFIG_REGISTER, (byte) (1 << RF24.MASK_RX_DR));
        System.out.println(rf24.printDetails());

        Receiver();

    }

    public void Receiver() throws PigpioException {


        p("***** Switching to RECEIVER role");
        cmd = "";
        role = 1;
        rf25.stopListening();
        rf25.powerDown();
       // rf25.openReadingPipe(1, rcvAddr);
       //0 rf25.openWritingPipe(sndAddr);
        System.out.println(rf25.printDetails());
        //rf25.startListening();

        p("***** Switching to RECEIVER role");
        cmd = "";
        role = 1;
        rf24.powerDown();
        rf24.openReadingPipe(1, rcvAddr);
       rf24.openWritingPipe(sndAddr);
        System.out.println(rf24.printDetails());
        rf24.startListening();

        while (1 == 1) {


            if (rf24.available()) {

                rf24.read(data32);
                //System.out.println("Received : "+Utils.dumpData(data32));
//                System.out.println(
//                        " B " +
//                                ((int) data32[0] * 256 + (int) data32[1]) +
//                                "\r\n X "
//                                + ((int) data32[2] * 256 + (int) data32[3]) +
//                                "\r\n Y "
//                                + ((int) data32[4] * 256 + (int) data32[5])+"\r\n"
//                );


            //    if ((X_temp < 2500 || X_temp> 3200) || (Y_temp < 2500 || Y_temp> 3200  ))

                //rf24.read(data32);
                rf24.stopListening();

              //  System.out.println("data"+data32);
               // while (rf24.available()) {
               //     rf24.read(datatemp);

                    //System.out.println("temnp"+datatemp);
             //   }

               // System.out.println("Dome.");
                    rf24.write(data32);
                X_temp = (int) data32[2] * 256 + (int) data32[3];
                Y_temp = (int) data32[4] * 256 + (int) data32[5];
                System.out.println(X_temp);
                System.out.println(Y_temp);
                System.out.print("seconds = 0.");
                System.out.println((System.currentTimeMillis()-oldMs)/100
                );
                oldMs= System.currentTimeMillis();
                countMs++;
                System.out.print("countMs = ");
                System.out.println(countMs);


                    rf24.startListening();




            }
//            else {
//                //p("Waiting for sender..." + counter++);
//                try { Thread.sleep(10);} catch (InterruptedException e){}
//            }
        }
//        rf24.terminate();
//        pigpio.gpioTerminate();
//
//        System.out.println("Done.");


    }


    private void p(String text) {
        System.out.println(text);
    }


}
