package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ManipulatorIOReal implements ManipulatorIO {
    private final DoubleSolenoid pushPiston;
    private final DoubleSolenoid holdPiston;

    public ManipulatorIOReal() {
        // TODO find out channels for manipulator pistons when electrical has them
        // TODO single solenoids not double
        this.pushPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2000, 2100);
        this.holdPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2200, 2300);
    }

    @Override
    public void pushCoral(final boolean extend) {
        if (extend) {
            pushPiston.set(DoubleSolenoid.Value.kReverse);
        } else {
            pushPiston.set(DoubleSolenoid.Value.kForward);
        }
    }

    @Override
    public void holdCoral(final boolean extend) {
        if (extend) {
            holdPiston.set(DoubleSolenoid.Value.kReverse);
        } else {
            holdPiston.set(DoubleSolenoid.Value.kForward);
        }
    }

    @Override
    public void updateInputs(final ManipulatorIOInputs inputs) {
        inputs.isCoralPresent = false;
    }
}
