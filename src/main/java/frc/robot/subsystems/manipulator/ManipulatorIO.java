package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public interface ManipulatorIO {
    ManipulatorIO EMPTY = new ManipulatorIO() {};

    @AutoLog
    class ManipulatorIOInputs {
        boolean isCoralPresent;
    }

    default void updateInputs(ManipulatorIOInputs inputs) {}

    default void pushCoral(boolean extend) {
        Logger.recordOutput("Manipulator/PushPiston", extend);
    }

    default void holdCoral(boolean extend) {
        Logger.recordOutput("Manipulator/HoldPiston", extend);
    }
}
