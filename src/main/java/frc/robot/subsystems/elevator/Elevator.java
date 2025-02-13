package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    ElevatorIO io;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ProfiledPIDController elevatorPIDController;
    Setpoint goal;
    boolean usePID;

    public Elevator(ElevatorIO io){
        this.io = io;
        elevatorPIDController = new ProfiledPIDController(.01, 0, 0, new TrapezoidProfile.Constraints(100, 1000));
        SmartDashboard.putData("Elevator PID", elevatorPIDController);
    }

    public enum Setpoint {
        //magic numbers
        Bottom(0.0),
        L1(5),
        L2(10),
        L3(24),
        L4(48);

        double value;

        Setpoint(double value) {
            this.value = value;
        }

        public Setpoint nextUp(){
            return switch(this){
                case Bottom -> L1;
                case L1 -> L2;
                case L2 -> L3;
                case L3 -> L4;
                case L4 -> L4;
              };
        }
        public Setpoint nextDown(){
            return switch(this){
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
        Logger.processInputs("Elevator", inputs); // ??? intake

        double outputPID = elevatorPIDController.calculate(inputs.currentHeight.in(Inches));
        if(usePID){
        io.setElevatorSpeed(outputPID); 

        
        }
    }

    public Command manualControl(DoubleSupplier speed){
        return run(()-> {
            usePID = false;
            io.setElevatorSpeed(speed.getAsDouble());
        }).finallyDo(()-> usePID = true).ignoringDisable(true);
    }

    
    public Command goToSetpoint(Setpoint setpoint){
        return goToSetpoint(()-> setpoint);
    }

    public Command goToSetpoint(Supplier<Setpoint> setpointSupplier){
        return runOnce(()->{
            goal = setpointSupplier.get();
            elevatorPIDController.setGoal(goal.value);
        })      
        .andThen(run(() -> {})
        .until(this::isAtSetpoint));
    }

    public Setpoint getSetPoint(){
        return goal;
    }

    public boolean isAtSetpoint() {
        return elevatorPIDController.atGoal();
    }
}
