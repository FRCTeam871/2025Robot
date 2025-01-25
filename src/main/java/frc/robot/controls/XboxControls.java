package frc.robot.controls;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class XboxControls implements IControls {
    CommandXboxController xboxController;

    public XboxControls() {
        xboxController = new CommandXboxController(0);
    }

    @Override
    public DoubleSupplier fowardsAndBackAxis() {
        return () -> Constants.deadbandAndExponential(xboxController.getLeftY());
    }

    @Override
    public DoubleSupplier sideToSideAxis() {
        return () -> Constants.deadbandAndExponential(xboxController.getLeftX());
    }

    @Override
    public DoubleSupplier driveRotation() {
        return () -> Constants.deadbandAndExponential(xboxController.getRightX());
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
        return xboxController.a();
    }

    @Override
    public Trigger elevatorMoveToSetPoint() {
        return xboxController.povUp();
    }    
}
