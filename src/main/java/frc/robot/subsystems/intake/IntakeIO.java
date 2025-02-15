package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public interface IntakeIO {
    IntakeIO EMPTY = new IntakeIO() {};

    @AutoLog
    class IntakeIOInputs {
        boolean isTargetValid;
        boolean tiltedRight;
        long timeStamp;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setLeftPistonOut(boolean extend) {
        Logger.recordOutput("Intake/LeftPiston", extend);
    }

    default void setRightPistonOut(boolean extend) {
        Logger.recordOutput("Intake/RightPiston", extend);
    }
}
