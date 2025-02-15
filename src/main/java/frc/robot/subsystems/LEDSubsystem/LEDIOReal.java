package frc.robot.subsystems.LEDSubsystem;

import frc.robot.subsystems.peripheral.Teensy;
import frc.robot.subsystems.peripheral.TeensyPacket;

public class LEDIOReal implements LEDIO {
    private final Teensy teensy;

    public LEDIOReal() {
        this.teensy = new Teensy(null);
    }

    @Override
    public void writeAllLED(TeensyPacket robert) {
        teensy.writeAllLED(robert);
    }
}
