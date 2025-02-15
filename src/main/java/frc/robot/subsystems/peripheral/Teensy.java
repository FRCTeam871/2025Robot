package frc.robot.subsystems.peripheral;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.subsystems.peripheral.io.ICommunicationsInterface;

/**
 * Supports PeripheralController version 2.X
 * Last updated for PeripheralController version 2.3.0
 */
public class Teensy {
    public static final String LED_PREFIX = "L";
    public static final String SOUND_PREFIX = "S";
    public static final String DELIMITER = ";";
    public static final String ALL_PREFIX = "A";
    public static final int MAX_MSG_LENGTH = 200;
    public static final int RAINBOW = -1;
    public static final int BINARY_COUNT = -1;

    // private ICommunicationsInterface comms;
    // private Deque<String> queue = new ArrayDeque<>();

    //    I2C i = new I2C(I2C.Port.kMXP, 0);
    SerialPort p = new SerialPort(9600, SerialPort.Port.kMXP);

    public Teensy(ICommunicationsInterface comms) {
        // this.comms = comms;
    }

    public void writeAllLED(TeensyPacket packet) {
        writeRaw(LED_PREFIX + ALL_PREFIX + DELIMITER + packet.getPayload());
    }

    public void writeLED(int strip, TeensyPacket packet) {
        writeRaw(LED_PREFIX + strip + DELIMITER + packet.getPayload());
    }

    public void writeRaw(String str) {
        if (str.length() > MAX_MSG_LENGTH) {
            System.err.println(
                    "[Teensy] Message exceeds max length! (" + str.length() + "/" + MAX_MSG_LENGTH + "): " + str);
        } else {
            // System.out.println("write \"" + str + "\n\"");
            p.writeString(str + "\n");
            p.flush();
            //            i.writeBulk((str + "\n").getBytes());
            //            queue.add(str);
        }
    }

    public static int color(int r, int g, int b) {
        int rgb = r;
        rgb = (rgb << 8) + g;
        rgb = (rgb << 8) + b;
        return rgb;
    }

    public static String hex(int color) {
        return String.format("0x%06X", 0xFFFFFF & color);
    }
}
