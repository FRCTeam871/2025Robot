package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class XboxControls implements IControls {
    CommandXboxController xboxController;
    CommandXboxController xboxController2;

    public XboxControls() {
        xboxController = new CommandXboxController(0);
        xboxController2 = new CommandXboxController(1);
    }

    @Override
    public DoubleSupplier fowardsAndBackAxis() {
        return () -> Constants.deadbandAndExponential(-xboxController.getLeftY());
    }

    @Override
    public DoubleSupplier sideToSideAxis() {
        return () -> Constants.deadbandAndExponential(-xboxController.getLeftX());
    }

    @Override
    public DoubleSupplier driveRotation() {
        return () -> Constants.deadbandAndExponential(-xboxController.getRightX());
    }

    @Override
    public Trigger manualElevatorMoveUp() {
        return xboxController.rightBumper();
    }

    @Override
    public Trigger manualElevatorMoveDown() {
        return xboxController.leftBumper();
    }

    @Override
    public Trigger goToNearestAprilTag() {
        return xboxController.b();
    }

    @Override
    public Trigger elevatorMoveToSetPoint() {
        return xboxController.povUp();
    }

    @Override
    public Trigger cancel() {
        return xboxController.a();
    }

    @Override
    public Trigger placeCoral() {
        return xboxController.x();
    }

    @Override
    public DoubleSupplier elevatorMove() {
        return () -> Constants.deadband(-xboxController2.getRightY(), .1);
    }
}
