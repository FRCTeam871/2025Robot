package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public interface IControls {
    // positive is fowards
    public DoubleSupplier fowardsAndBackAxis();
    // positive is left
    public DoubleSupplier sideToSideAxis();
    // positive is counterclockwise
    public DoubleSupplier driveRotation();
    // positive is up
    public DoubleSupplier elevatorMove();

    public Trigger manualElevatorMoveUp();

    public Trigger manualElevatorMoveDown();

    public Trigger goToNearestAprilTag();

    public Trigger intakePiston1();

    public Trigger intakePiston2();

    public Trigger placeCoral();

    public Trigger cancel();

    public Trigger switchManualElevator();

    public Trigger pushCoral();

    public Trigger releaseCoral();

    public Trigger compressorToggle();

    public Trigger fieldOrientationToggle();

    public Trigger buttonL1();

    public Trigger buttonL2();

    public Trigger buttonL3();

    public Trigger buttonL4();
}
