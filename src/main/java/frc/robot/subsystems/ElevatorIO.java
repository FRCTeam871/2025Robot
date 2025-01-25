package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        Distance currentHeight;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}
    public default void setElevatorSpeed(double speed){
        Logger.recordOutput("Elevator/Speed",speed);
    }
}
