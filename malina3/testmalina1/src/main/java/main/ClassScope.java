package main;

import java.util.Vector;

public class ClassScope {
    public static String[] getLoadedLibraries(final ClassLoader loader) throws NoSuchFieldException, IllegalAccessException {
        final java.lang.reflect.Field LIBRARIES;
        LIBRARIES = ClassLoader.class.getDeclaredField("loadedLibraryNames");
        LIBRARIES.setAccessible(true);
        final Vector<String> libraries = (Vector<String>) LIBRARIES.get(loader);
        return libraries.toArray(new String[] {});
    }
}
