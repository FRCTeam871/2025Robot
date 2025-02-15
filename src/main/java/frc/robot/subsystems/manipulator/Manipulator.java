package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.fieldtracking.FieldTracking;

public class Manipulator extends SubsystemBase {
    private final ManipulatorIO io;
    private final FieldTracking fieldTracking;
    public Manipulator(final ManipulatorIO io, final FieldTracking fieldTracking) {
        this.io = io;
        this.fieldTracking = fieldTracking;
    }

    public Command pushCoral() {
        return Commands.run(() -> io.pushCoral(true))
                .finallyDo(canceled -> io.pushCoral(false));
        
    }

    public Command releaseCoral() {
        return Commands.run(() -> io.holdCoral(false))
                .finallyDo(canceled -> io.holdCoral(true));
    }

    public Command scoreCoral() {
        if (fieldTracking.followAprilTag().isFinished()) {
            return run(() -> releaseCoral().withTimeout(.5).andThen(pushCoral().withTimeout(.5)));
        } else {
            return run(() -> {});
        }
    }
}
