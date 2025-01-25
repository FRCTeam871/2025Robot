package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface FieldTrackingIO {
    @AutoLog
    public static class FieldTrackingIOInputs{
        int tagCount;
        Pose2d pose;
        double timestampSeconds;
        long tid;
        double[] targetpose_robotspace;
    }
    public default void updateInputs(FieldTrackingIOInputs inputs) {}
    public default void setRobotOrientation(double degrees){}
}
