package frc.robot.controls;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IControls {
    //positive is fowards
    public DoubleSupplier fowardsAndBackAxis();
    //positive is left
    public DoubleSupplier sideToSideAxis();
    //positive is counterclockwise
    public DoubleSupplier driveRotation();
    //positive is up
    public DoubleSupplier elevatorMove();

    public Trigger manualElevatorMoveUp();
    
    public Trigger manualElevatorMoveDown();
    
    public Trigger goToNearestAprilTag();

    public Trigger elevatorMoveToSetPoint();

   public Trigger placeCoral();

   public Trigger cancel();
}
