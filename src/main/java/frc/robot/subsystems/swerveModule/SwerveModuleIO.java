package frc.robot.subsystems.swerveModule;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs{
        double steeringAngleDegrees;
        double driveVelocity;
        double drivePosition;
    }
    public default void updateInputs(SwerveModuleIOInputs inputs) {}
    public default void setDriveSpeed(double speed){}
    public default void setSteerSpeed(double speed){}


} 
