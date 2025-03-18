package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ManipulatorIOReal implements ManipulatorIO {
    private final Solenoid pushPiston;
    private final Solenoid holdPiston;
    private final DigitalInput coralSensor;

    public ManipulatorIOReal() {
        this.pushPiston = new Solenoid(1, PneumaticsModuleType.CTREPCM, 0);
        this.holdPiston = new Solenoid(1, PneumaticsModuleType.CTREPCM, 1);
        this.coralSensor = new DigitalInput(9);
    }

    @Override
    public void pushCoral(final boolean extend) {
        ManipulatorIO.super.pushCoral(extend);
        pushPiston.set(extend);
    }

    @Override
    public void holdCoral(final boolean extend) {
        ManipulatorIO.super.holdCoral(extend);
        holdPiston.set(!extend);
    }

    @Override
    public void updateInputs(final ManipulatorIOInputs inputs) {
        inputs.isCoralPresent = coralSensor.get();
    }
}
