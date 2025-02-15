package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        boolean isTargetValid;
        boolean tiltedRight;
        long timeStamp;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setLeftPistonOut(boolean extend) {
        Logger.recordOutput("Intake/LeftPiston", extend);
    }

    public default void setRightPistonOut(boolean extend) {
        Logger.recordOutput("Intake/RightPiston", extend);
    }
}
