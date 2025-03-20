package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class XboxControls implements IControls {

    CommandXboxController driveXboxController;
    CommandXboxController systemXboxController;

    public XboxControls() {
        driveXboxController = new CommandXboxController(0); // xboxcontroller 1
        systemXboxController = new CommandXboxController(1); // xbox controlr 2
    }

    @Override
    public DoubleSupplier fowardsAndBackAxis() {
        return () -> Constants.deadbandAndExponential(-driveXboxController.getLeftY());
    }

    @Override
    public DoubleSupplier sideToSideAxis() {
        return () -> Constants.deadbandAndExponential(-driveXboxController.getLeftX());
    }

    @Override
    public DoubleSupplier driveRotation() {
        return () -> Constants.deadbandAndExponential(-driveXboxController.getRightX())*.6;
    }

    @Override
    public Trigger manualElevatorMoveUp() {
        return driveXboxController.rightBumper();
    }

    @Override
    public Trigger manualElevatorMoveDown() {
        return driveXboxController.leftBumper();
    }

    @Override
    public Trigger goToNearestAprilTag() {
        return driveXboxController.povRight();
    }

    @Override
    public Trigger cancel() {
        return driveXboxController.b();
    }

    @Override
    public Trigger placeCoral() {
        return driveXboxController.x();
    }

    @Override
    public DoubleSupplier elevatorMove() {
        return () -> Constants.deadband(-systemXboxController.getRightY(), .1);
    }

    @Override
    public Trigger switchManualElevator() {
        return systemXboxController.start();
    }

    @Override
    public Trigger intakePiston1() {
        return systemXboxController.a();
    }

    @Override
    public Trigger intakePiston2() {
        return systemXboxController.b();
    }

    @Override
    public Trigger pushCoral() {
        return driveXboxController.y();
    }

    @Override
    public Trigger releaseCoral() {
        return driveXboxController.a();
    }

    @Override
    public Trigger compressorToggle() {
        return driveXboxController.back();
    }

    @Override
    public Trigger fieldOrientationToggle() {
        return systemXboxController.b();
    }

    @Override
    public Trigger buttonL1() {
        return driveXboxController.povDown();
    }

    @Override
    public Trigger buttonL2() {
        return driveXboxController.povLeft();
    }

    @Override
    public Trigger buttonL3() {
        return driveXboxController.povRight();
    }

    @Override
    public Trigger buttonL4() {
        return driveXboxController.povUp();
    }
}
