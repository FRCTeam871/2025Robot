package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveDriveIO {
    @AutoLog
    public static class SwerveDriveIOInputs {
        Rotation2d gyroRotation = new Rotation2d(0);
        boolean isCalibrating;
        double gyroRate;
    }

    public default void updateInputs(SwerveDriveIOInputs inputs) {}
}
