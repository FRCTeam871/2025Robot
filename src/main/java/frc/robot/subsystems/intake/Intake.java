package frc.robot.subsystems.intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.Setpoint;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private long detectionStartTime;
    private long detectionEndTime;
    private boolean wasTargetValid;
    Elevator elevator;
    // Command
    public Intake(IntakeIO io, Elevator elevator) {
        this.elevator = elevator;
        this.io = io;
        setDefaultCommand(dislodge().ignoringDisable(true));
        io.setLeftPistonOut(false);
        io.setRightPistonOut(false);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public Command sendLeftPistonOut() {
        return Commands.run(() -> io.setLeftPistonOut(true))
                .finallyDo(canceled -> io.setLeftPistonOut(false))
                .ignoringDisable(true);
    }

    public Command sendRightPistonOut() {
        return Commands.run(() -> io.setRightPistonOut(true))
                .finallyDo(canceled -> io.setRightPistonOut(false))
                .ignoringDisable(true);
    }

    public Command dislodge() {
        return run(() -> {
            if (elevator.getSetPoint() == Setpoint.Bottom) {
                // if target newly detected
                if (inputs.isTargetValid && !wasTargetValid) {
                    // reset start time only if not detected for n micros
                    if (inputs.timeStamp - detectionEndTime > Constants.TARGET_DROP_THRESHOLD.in(Units.Microsecond)) {
                        detectionStartTime = inputs.timeStamp;
                    }
                }
                // if target newly lost set end time
                if (!inputs.isTargetValid && wasTargetValid) {
                    detectionEndTime = inputs.timeStamp;
                }

                // Logger.recordOutput("Intake/StartTime", detectionStartTime);
                // Logger.recordOutput("Intake/EndTime", detectionEndTime);
                // Logger.recordOutput("Intake/LostDuration", (double) (inputs.timeStamp - detectionEndTime) / 1e6);
                // Logger.recordOutput("Intake/DetectDuration", (double) (inputs.timeStamp - detectionStartTime) / 1e6);
                // if we have target and amount of time is passed
                if (inputs.isTargetValid
                        && inputs.timeStamp - detectionStartTime > Constants.PISTON_THRESHOLD.in(Units.Microsecond)) {
                    // trigger pistons
                    if (inputs.tiltedRight) {
                        sendRightPistonOut()
                                .withTimeout(Constants.PISTON_OUT_TIME)
                                .schedule();
                    } else {
                        sendLeftPistonOut()
                                .withTimeout(Constants.PISTON_OUT_TIME)
                                .schedule();
                    }

                    detectionStartTime = inputs.timeStamp;
                }

                wasTargetValid = inputs.isTargetValid;
            } else {

            }
        });
    }
}
