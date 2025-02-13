package frc.robot.subsystems.fieldtracking;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.fieldtracking.FieldTrackingIO.IMUMode;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class FieldTracking extends SubsystemBase {
    ProfiledPIDController sidePidController;
    ProfiledPIDController forwardPidController;
    ProfiledPIDController yawPidController;
    SwerveDrive swerveDrive;
    FieldTrackingIO io;
    FieldTrackingIOInputsAutoLogged inputs = new FieldTrackingIOInputsAutoLogged();
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public FieldTracking(final SwerveDrive swerveDrive, final FieldTrackingIO io) {
        sidePidController = new ProfiledPIDController(2.5, 0.001, .1, new Constraints(1000, 1000));
        forwardPidController = new ProfiledPIDController(2.5, 0.001, .1, new Constraints(1000, 1000));
        yawPidController = new ProfiledPIDController(0.08, 0.001, 0, new Constraints(1000, 1000));
        this.swerveDrive = swerveDrive;
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("FieldTracking", inputs);

        io.setRobotOrientation(swerveDrive.getEstimatedPose().getRotation().getDegrees());            
        // if our angular velocity is greater than 720 degrees per
        // second, ignore vision updates
        if (Math.abs(swerveDrive.getYawRate()) > 720) {
            Logger.recordOutput("FieldTracking/TargetPoses", new Pose3d[]{});
            Logger.recordOutput("FieldTracking/TargetIDs", new int[]{});
        } else if (inputs.tagCount == 0) {
            Logger.recordOutput("FieldTracking/TargetPoses", new Pose3d[]{});
            Logger.recordOutput("FieldTracking/TargetIDs", new int[]{});

        } else {
            Optional<Pose3d> pose= fieldLayout.getTagPose((int)inputs.tid);
            if(pose.isPresent()){
                swerveDrive.addVisionMeasurement(
                        inputs.pose,
                        inputs.timestampSeconds,
                        VecBuilder.fill(.7, .7, 9999999)
                        );

                Logger.recordOutput("FieldTracking/TargetPoses", new Pose3d[]{pose.get()});
                Logger.recordOutput("FieldTracking/TargetIDs", new int[]{(int)inputs.tid});
            } else {
                Logger.recordOutput("FieldTracking/TargetPoses", new Pose3d[]{});
                Logger.recordOutput("FieldTracking/TargetIDs", new int[]{});
            }
        }
    }

    public boolean isAprilTagDetected() {
        // maybe convert to int?
        return inputs.tid != -1;
    }

    public long getAprilTag(){
        return inputs.tid;
    }
    // public Command followAprilTag() {
    // return run(() -> {
    // if (!isAprilTagDetected()) {
    // ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
    // swerveDrive.updateSpeed(speeds); // this funciton sets everything to 0
    // return;
    // }

    public Command followAprilTag() {
        return run(() -> {
            if (!isAprilTagDetected()) {
                ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
                swerveDrive.updateSpeed(speeds);
                return;
            } else {

                // step one read output from limelight

                // in meters

                double tx = -inputs.targetpose_robotspace[0];
                Logger.recordOutput("FieldTracking/tx", tx);
                double ty = inputs.targetpose_robotspace[1];
                Logger.recordOutput("FieldTracking/ty", ty);
                double tz = inputs.targetpose_robotspace[2];
                Logger.recordOutput("FieldTracking/tz", tz);
                // in degrees
                double pitch = inputs.targetpose_robotspace[3];
                Logger.recordOutput("FieldTracking/pitch", pitch);
                double yaw = inputs.targetpose_robotspace[4];
                Logger.recordOutput("FieldTracking/yaw", yaw);
                double roll = inputs.targetpose_robotspace[5];
                Logger.recordOutput("FieldTracking/roll", roll);
                // step two feed values into pids
                double xout = sidePidController.calculate(-tx, 0);
                double zout = forwardPidController.calculate(-tz, 1);
                double yawout = yawPidController.calculate(-yaw, 0);

                // step three take pid values and put it into swervedrive
                ChassisSpeeds speeds = new ChassisSpeeds(zout, xout, yawout);

                swerveDrive.updateSpeed(speeds); // this will update the speeed

            }
        })
                .until(this::isAtPosition);
    }

    public Command maintainPose(Pose2d poseToMaintain) {
        return run(() -> {
            Logger.recordOutput("FieldTracking/MaintainPose", poseToMaintain);

            Pose2d relTgtPose = poseToMaintain.relativeTo(swerveDrive.getEstimatedPose());

            double yout = sidePidController.calculate(0, relTgtPose.getY());
            Logger.recordOutput("FieldTracking/tyPID", yout);
            double xout = forwardPidController.calculate(0, relTgtPose.getX());
            Logger.recordOutput("FieldTracking/txPID", xout);
            double yawout = yawPidController.calculate(0, relTgtPose.getRotation().getDegrees());
            ChassisSpeeds speeds = new ChassisSpeeds(xout, yout, yawout);
            swerveDrive.updateSpeed(speeds); // this will update the speeed
        });
    }

    public boolean isAtPosition() {
        return sidePidController.atGoal() && forwardPidController.atGoal() && yawPidController.atGoal();
    }

    public void setCameraIMUMode(IMUMode imuMode) {
        io.setCameraIMUMode(imuMode);

    }

    public void setThrottle(int throttle) {
        io.setCameraThrottle(throttle);
    }
}
