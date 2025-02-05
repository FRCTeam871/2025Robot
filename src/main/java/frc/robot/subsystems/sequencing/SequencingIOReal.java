
package frc.robot.subsystems.sequencing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SequencingIOReal implements SequencingIO {
    public SequencingIOReal(){
        SmartDashboard.putBoolean("leftL1",false);
        SmartDashboard.putBoolean("leftL2",false);
        SmartDashboard.putBoolean("leftL3",false);
        SmartDashboard.putBoolean("leftL4",false);
        SmartDashboard.putBoolean("rightL1",false);
        SmartDashboard.putBoolean("rightL2",false);
        SmartDashboard.putBoolean("rightL3",false);
        SmartDashboard.putBoolean("rightL4",false);
    }
    @Override
    public void updateInputs(SequencingIOInputs inputs) {
        inputs.leftL1 = SmartDashboard.getBoolean("leftL1", false);
        inputs.leftL2 = SmartDashboard.getBoolean("leftL2", false);
        inputs.leftL3 = SmartDashboard.getBoolean("leftL3", false);
        inputs.leftL4 = SmartDashboard.getBoolean("leftL4", false);
        inputs.rightL1 = SmartDashboard.getBoolean("rightL1", false);
        inputs.rightL2 = SmartDashboard.getBoolean("rightL2", false);
        inputs.rightL3 = SmartDashboard.getBoolean("rightL3", false);
        inputs.rightL4 = SmartDashboard.getBoolean("rightL4", false);
    }
}