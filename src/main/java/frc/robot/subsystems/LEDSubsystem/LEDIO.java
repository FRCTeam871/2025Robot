package frc.robot.subsystems.LEDSubsystem;

import frc.robot.subsystems.peripheral.TeensyPacket;

public interface LEDIO {
    LEDIO EMPTY = new LEDIO() {};

    default void writeAllLED(final TeensyPacket robert) {}
}
