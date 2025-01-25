package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Elevator extends SubsystemBase{
    ElevatorIO io;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ProfiledPIDController elevatorPIDController;
    double goal;

    public Elevator(ElevatorIO io){
        this.io = io;
        elevatorPIDController = new ProfiledPIDController(.01, 0, 0, new TrapezoidProfile.Constraints(100, 1000));
    }
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        double outputPID = elevatorPIDController.calculate(inputs.currentHeight.in(Inches));
    
        io.setElevatorSpeed(outputPID); 
    }

    
    public Command goToSetPoint(double setpoint){
        return runOnce(()->{
            elevatorPIDController.setGoal(setpoint);
            goal = setpoint;
        })      
        .andThen(run(() -> {})
        .until(this::isAtSetpoint));
    }

    public double getSetPoint(){
        return elevatorPIDController.getGoal().position;
    }

    public boolean isAtSetpoint() {
        return elevatorPIDController.atGoal();
    }
}
