package main;



public class JNIHelloWorld {
    static {
        // $PROJECT_ROOT -- абсолютный путь до библиотеки
        System.out.println(System.getProperty("java.library.path"));
        System.loadLibrary("helloworld");
    }

    native void printHelloWorld();
    public static void main(String[] args) {
        System.setProperty("file.encoding", "UTF-8");
        try {
            final String[] libraries = ClassScope.getLoadedLibraries(ClassLoader.getSystemClassLoader());
            if (libraries.length<1){
                System.out.println("no libs");
            }
            for (String lib: libraries) {
                System.out.println(lib);
            }
        } catch (NoSuchFieldException e) {
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        JNIHelloWorld p = new JNIHelloWorld();
        p.printHelloWorld();
    }

}
