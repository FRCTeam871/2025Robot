package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.fieldtracking.FieldTracking;

public class Manipulator extends SubsystemBase {
    private final ManipulatorIO io;
    private final FieldTracking fieldTracking;

    public Manipulator(final ManipulatorIO io, final FieldTracking fieldTracking) {
        this.io = io;
        this.fieldTracking = fieldTracking;
    }

    public Command sendPushPistonIn() {
        return run(() -> io.pushCoral(true))
                .finallyDo(canceled -> io.pushCoral(false))
                .withTimeout(.5);
    }

    public Command sendHoldPistonIn() {
        return run(() -> io.holdCoral(true))
                .finallyDo(canceled -> io.holdCoral(false))
                .withTimeout(.5);
    }

    public Command scoreCoral() {
        if (fieldTracking.followAprilTag().isFinished()) {
            return run(() -> sendHoldPistonIn().andThen(sendPushPistonIn()));
        } else {
            return run(() -> {});
        }
    }
}
