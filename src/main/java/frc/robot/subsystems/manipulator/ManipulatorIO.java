package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public interface ManipulatorIO {
    @AutoLog
    public static class ManipulatorIOInputs {
        boolean isCoralPresent;
    }

    public default void updateInputs(ManipulatorIOInputs inputs) {}

    public default void pushCoral(boolean extend) {
        Logger.recordOutput("Manipulator/PushPiston", extend);
    }

    public default void holdCoral(boolean extend) {
        Logger.recordOutput("Manipulator/HoldPiston", extend);
    }
}
