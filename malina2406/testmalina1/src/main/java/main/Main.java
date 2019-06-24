package main;

import Module.Joystick;
import jpigpio.*;
import rf24j.RF24;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

public class Main {

    public static void main(String[] args)throws PigpioException {


        DataExchange datatest = new DataExchange();
        datatest.Receiver();

        Joystick joys =  new Joystick();

    }
}
