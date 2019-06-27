package Module;


/**
 * callback function when data received from FR
 * @author Alex maoanapex88@163.com
 */
public interface ReceiveListener {

    /**
     * @param data data bytes arrived
     */
    void dataReceived(int[] data);

}