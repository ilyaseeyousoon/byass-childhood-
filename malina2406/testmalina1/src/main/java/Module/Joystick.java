package Module;

public class Joystick extends Module {

    int X=9999;
    int Y=9999;
    boolean Button=false;

    public int getX() {
        return X;
    }

    public void setX(int x) {
        X = x;
    }

    public int getY() {
        return Y;
    }

    public void setY(int y) {
        Y = y;
    }

    public boolean isButton() {
        return Button;
    }

    public void setButton(boolean button) {
        Button = button;
    }

}
