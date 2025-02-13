package frc.robot.subsystems.LEDSubsystem;

import frc.robot.subsystems.peripheral.TeensyPacket;

public interface LEDIO {
public default void writeAllLED(TeensyPacket robert) {}
}