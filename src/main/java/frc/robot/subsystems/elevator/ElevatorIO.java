package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        Distance currentHeight = Units.Inch.of(0);
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
        if (!SmartDashboard.containsKey("Elevator/Height")) {
            SmartDashboard.putNumber("Elevator/Height", 0);
        }
        inputs.currentHeight = Units.Inch.of(SmartDashboard.getNumber("Elevator/Height",  0));
    }

    public default void setElevatorSpeed(double speed){
        Logger.recordOutput("Elevator/Speed",speed);
    }
}
