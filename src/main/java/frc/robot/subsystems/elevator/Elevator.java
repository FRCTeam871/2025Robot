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
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ProfiledPIDController elevatorPIDController;
    private Setpoint goal = Setpoint.Bottom;
    private boolean usePID = true;

    public enum Setpoint {
        // very magical numbers (inches)
        Bottom(18.25),
        L1(18.25),
        L2(30),
        L3(49),
        L4(74), // good enuf
        ClimbingMount(77.25); // good enuf

        final double value;

        Setpoint(final double value) {
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

    public Elevator(final ElevatorIO io) {
        this.io = io;
        this.elevatorPIDController = new ProfiledPIDController(.1, 0, 0.02, new TrapezoidProfile.Constraints(100, 250));
        elevatorPIDController.setGoal(goal.value);
        SmartDashboard.putData("Elevator/PID", elevatorPIDController);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        double outputPID = 0;
        if (usePID) {
            outputPID = elevatorPIDController.calculate(inputs.currentHeight.in(Inches));
            io.setElevatorSpeed(outputPID);
        }
        Logger.recordOutput("Elevator/usePID", usePID);
        Logger.recordOutput("Elevator/PID", outputPID);
    }

    public Command manualControl(final DoubleSupplier speed) {
        return run(() -> {
                    usePID = false;
                    io.setElevatorSpeed(speed.getAsDouble());
                })
                .finallyDo(() -> usePID = true)
                .ignoringDisable(true);
    }

    public Command goToSetpoint(final Setpoint setpoint) {
        return goToSetpoint(() -> setpoint);
    }

    public Command goToSetpoint(final Supplier<Setpoint> setpointSupplier) {
        return runOnce(() -> {
            // usePID = true;
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
    public double getCurrentHeightNormalized(){
        return inputs.currentHeightNormalized;
    }
}
