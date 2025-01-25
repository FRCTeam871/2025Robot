package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ManipulatorIOReal implements ManipulatorIO {
    DoubleSolenoid pushPiston;
    DoubleSolenoid holdPiston;
    public ManipulatorIOReal(){
        //TODO find out channels for manipulator pistons when electrical has them
        pushPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 20, 21);
        holdPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 22, 23);
    }
    @Override
    public void pushCoral(boolean extend) {
        if (extend) {
            pushPiston.set(DoubleSolenoid.Value.kReverse);
        } else {
            pushPiston.set(DoubleSolenoid.Value.kForward);
        }
    }

    @Override
    public void holdCoral(boolean extend) {
        if (extend) {
            holdPiston.set(DoubleSolenoid.Value.kReverse);
        } else {
            holdPiston.set(DoubleSolenoid.Value.kForward);
        }
    }
}
