package frc.robot.subsystems.peripheral;

public class LEDStripSettings {

    /**
     * @param brightness The brightness to set as a percentage [0-100]
     */
    public static TeensyPacket brightness(int brightness) {
        return new TeensyPacket("/BR" + Teensy.DELIMITER + brightness);
    }

    /**
     * @param reverse If the strip should be reversed (flipped).
     */
    public static TeensyPacket reverse(boolean reverse) {
        return new TeensyPacket("/RV" + Teensy.DELIMITER + (reverse ? 1 : 0));
    }
}
