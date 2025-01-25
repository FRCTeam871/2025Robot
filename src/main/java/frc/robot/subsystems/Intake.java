package frc.robot.subsystems;

import java.security.Timestamp;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    long detectionStartTime;
    long detectionEndTime;
    boolean wasTargetValid;

    public Intake(IntakeIO io) {
        this.io = io;

        setDefaultCommand(dislodge().ignoringDisable(true));
        
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public Command sendLeftPistonOut() {
        return run(() -> io.setLeftPistonOut(true)).finallyDo(canceled -> io.setLeftPistonOut(false)).ignoringDisable(true);
    }

    public Command sendRightPistonOut() {
        return run(() -> io.setRightPistonOut(true)).finallyDo(canceled -> io.setRightPistonOut(false)).ignoringDisable(true);
    }

    public Command dislodge() {
        return run(() -> {
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

            Logger.recordOutput("Intake/StartTime", detectionStartTime);
            Logger.recordOutput("Intake/EndTime", detectionEndTime);
            Logger.recordOutput("Intake/LostDuration", (double)(inputs.timeStamp - detectionEndTime) / 1e6);
            Logger.recordOutput("Intake/DetectDuration", (double)(inputs.timeStamp - detectionStartTime) / 1e6);
            // if we have target and amount of time is passed
            if (inputs.isTargetValid && inputs.timeStamp - detectionStartTime > Constants.PISTON_THRESHOLD.in(Units.Microsecond)) {
                // trigger pistons
                if (inputs.tiltedRight) {
                    sendRightPistonOut().withTimeout(Constants.PISTON_OUT_TIME).schedule();
                } else {
                    sendLeftPistonOut().withTimeout(Constants.PISTON_OUT_TIME).schedule();
                }
                
                detectionStartTime = inputs.timeStamp;
            }

            wasTargetValid = inputs.isTargetValid; 
        });
    }
}
