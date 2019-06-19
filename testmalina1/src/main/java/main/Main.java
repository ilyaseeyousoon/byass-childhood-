package main;

public class Main {

    public static void main(String[] args) {

        System.out.println(System.getProperty("java.library.path"));
        System.loadLibrary("JPigpioC");
        System.out.println("Test_RF24");
        //System.setProperty("java.library.path", "./home/pi/pigpio-master/jpigc/jpigpio/JPigpioC/");
        System.out.println(System.getProperty("java.library.path"));
        //System.load("/home/pi/pigpio-master/jpigc/jpigpio/JPigpioC/libJPigpioC.so");
        System.loadLibrary("JPigpioC");

        try {
            final String[] libraries = ClassScope.getLoadedLibraries(ClassLoader.getSystemClassLoader());
            for (String lib: libraries) {
                System.out.println(lib);
            }
        } catch (NoSuchFieldException e) {
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }

    }
}
