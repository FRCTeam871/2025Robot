package frc.robot.controls;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IControls {
    public DoubleSupplier fowardsAndBackAxis();

    public DoubleSupplier sideToSideAxis();

    public DoubleSupplier driveRotation();

    public Trigger manualElevatorMoveUp();
    
    public Trigger manualElevatorMoveDown();
    
    public Trigger goToNearestAprilTag();

    public Trigger elevatorMoveToSetPoint();

   
}
