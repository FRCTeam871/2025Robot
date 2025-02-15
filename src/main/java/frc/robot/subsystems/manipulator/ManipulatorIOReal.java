package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ManipulatorIOReal implements ManipulatorIO {
    private final Solenoid pushPiston;
    private final Solenoid holdPiston;

    public ManipulatorIOReal() {
        this.pushPiston = new Solenoid(1, PneumaticsModuleType.CTREPCM, 0);
        this.holdPiston = new Solenoid(1, PneumaticsModuleType.CTREPCM, 1);
    }

    @Override
    public void pushCoral(final boolean extend) {
        pushPiston.set(extend);
    }

    @Override
    public void holdCoral(final boolean extend) {
        holdPiston.set(!extend);
    }

    @Override
    public void updateInputs(final ManipulatorIOInputs inputs) {
        inputs.isCoralPresent = false;
    }
}