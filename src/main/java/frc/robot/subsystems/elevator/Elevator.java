package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    ElevatorIO io;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ProfiledPIDController elevatorPIDController;
    Setpoint goal = Setpoint.Bottom;
    boolean usePID;

    public Elevator(ElevatorIO io) {
        this.io = io;
        elevatorPIDController = new ProfiledPIDController(.1, 0, 0.02, new TrapezoidProfile.Constraints(100, 250));
        SmartDashboard.putData("Elevator/PID", elevatorPIDController);
        elevatorPIDController.setGoal(goal.value);
    }

    public enum Setpoint {

        // very magical numbers (inches)
        Bottom(18.25),
        L1(18.25),
        L2(30),
        L3(49),
        L4(74), // good enuf
        ClimbingMount(77.25); // good enuf

        double value;

        Setpoint(double value) {
            this.value = value;
        }

        public Setpoint nextUp() {
            return switch (this) {
                case Bottom -> L1;
                case L1 -> L2;
                case L2 -> L3;
                case L3 -> L4;
                case L4 -> ClimbingMount;
                case ClimbingMount -> ClimbingMount;
            };
        }

        public Setpoint nextDown() {
            return switch (this) {
                case ClimbingMount -> L4;
                case L4 -> L3;
                case L3 -> L2;
                case L2 -> L1;
                case L1 -> Bottom;
                case Bottom -> Bottom;
            };
        }
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs); // intake???

        double outputPID = elevatorPIDController.calculate(inputs.currentHeight.in(Inches));
        Logger.recordOutput("Elevator/PID", outputPID);
        if (usePID) {
            io.setElevatorSpeed(outputPID);
        }
    }

    public Command manualControl(DoubleSupplier speed) {
        return run(() -> {
                    usePID = false;
                    io.setElevatorSpeed(speed.getAsDouble());
                })
                .finallyDo(() -> usePID = true)
                .ignoringDisable(true);
    }

    public Command goToSetpoint(Setpoint setpoint) {
        return goToSetpoint(() -> setpoint);
    }

    public Command goToSetpoint(Supplier<Setpoint> setpointSupplier) {
        return runOnce(() -> {
            usePID = true;
            goal = setpointSupplier.get();
            elevatorPIDController.setGoal(goal.value);
        });
    }

    public Setpoint getSetPoint() {
        return goal;
    }

    public boolean isAtSetpoint() {
        return elevatorPIDController.atGoal();
    }
}
