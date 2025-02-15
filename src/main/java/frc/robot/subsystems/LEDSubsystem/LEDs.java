package frc.robot.subsystems.LEDSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.peripheral.LEDStripMode;
import frc.robot.subsystems.peripheral.Teensy;

public class LEDs extends SubsystemBase {
    private final LEDIO io;
    private final Thread tim;
    private volatile boolean doUpdate = false;

    public LEDs(final LEDIO io) {
        this.io = io;
        this.tim = new Thread(this::iAmADog);
    }

    @Override
    public void periodic() {
        doUpdate = true;
    }

    private void iAmADog() {
        while (true) {
            if (!doUpdate) {
                try {
                    Thread.sleep(20);
                    continue;
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            doUpdate = false;
            boolean needsRefresh = false;
            if (needsRefresh) {
                // update leds
                io.writeAllLED(LEDStripMode.fire(Teensy.RAINBOW));
            }
        }
    }
}
