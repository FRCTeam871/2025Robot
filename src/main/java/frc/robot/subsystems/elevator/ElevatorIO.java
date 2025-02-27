package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;

public interface ElevatorIO {
    ElevatorIO EMPTY = new ElevatorIO() {};

    @AutoLog
    class ElevatorIOInputs {
        Distance currentHeight = Units.Inch.of(0);
        Distance currentHeightRelative = Units.Inch.of(0);
        double currentHeightNormalized; 
    }

    default void updateInputs(ElevatorIOInputs inputs) {
        if (!SmartDashboard.containsKey("Elevator/Height")) {
            SmartDashboard.putNumber("Elevator/Height", 0);
        }
        inputs.currentHeight = Units.Inch.of(SmartDashboard.getNumber("Elevator/Height", 0));
    }

    default void setElevatorSpeed(double speed) {
        Logger.recordOutput("Elevator/Speed", speed);
    }

    default void setBrakeMode(boolean isBrakeOn){}

    default void resetRelativeEncoder(){

    }
}
