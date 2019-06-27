package Module;

public class Joystick extends Module {

    int X=9999;
    int Y=9999;
    boolean Button=false;

    public int getX() {


        return X;
    }

    protected void setX(int x) {
        X = x;
    }

    public int getY() {
        return Y;
    }

    protected void setY(int y) {
        Y = y;
    }

    public boolean isButton() {
        return Button;
    }

    protected void setButton(boolean button) {
        Button = button;
    }










}
