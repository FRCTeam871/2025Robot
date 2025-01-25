package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.FieldTrackingIO.FieldTrackingIOInputs;


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
