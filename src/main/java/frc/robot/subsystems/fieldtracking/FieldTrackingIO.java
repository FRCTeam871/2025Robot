package frc.robot.subsystems.fieldtracking;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface FieldTrackingIO {
    @AutoLog
    public static class FieldTrackingIOInputs {
        int tagCount;
        Pose2d pose = new Pose2d();
        double timestampSeconds;
        long tid;
        double[] targetpose_robotspace;
    }

    public enum IMUMode {
        /** Only external (navx) angle used */
        ExternalOnly(0),
        /** External angle used, internal angle reset to match */
        ExternalReset(1),
        /** Only internal angle used */
        InternalOnly(2),
        /** Only internal angle used, also factors in MT1 angle from apriltags */
        InternalMT1Assist(3),
        /** Internal angle used, also factors in external angle while not moving */
        InternalExternalAssist(4);

        public int limeLightConstant;

        private IMUMode(int limeLightConstant) {
            this.limeLightConstant = limeLightConstant;
        }

    }

    public default void updateInputs(FieldTrackingIOInputs inputs) {}
    public default void setRobotOrientation(double degrees) {}
    public default void setCameraIMUMode(IMUMode imuMode) {}
    public default void setCameraThrottle(int throttle) {}
}
