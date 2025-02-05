package frc.robot.subsystems.sequencing;

import org.littletonrobotics.junction.AutoLog;

public interface SequencingIO {
    @AutoLog
    public static class SequencingIOInputs {
        boolean leftL1;
        boolean leftL2;
        boolean leftL3;
        boolean leftL4;
        boolean rightL1;
        boolean rightL2;
        boolean rightL3;
        boolean rightL4;
    }
    public default void updateInputs(SequencingIOInputs inputs){}
}
