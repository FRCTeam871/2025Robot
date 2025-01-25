package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.FieldTrackingIO.FieldTrackingIOInputs;

public class FieldTracking extends SubsystemBase {
    ProfiledPIDController txPidController;
    ProfiledPIDController tzPidController;
    ProfiledPIDController yawPidController;
    SwerveDrive swerveDrive;
    FieldTrackingIO io;
    FieldTrackingIOInputsAutoLogged inputs = new FieldTrackingIOInputsAutoLogged();
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public FieldTracking(final SwerveDrive swerveDrive, final FieldTrackingIO io) {
        txPidController = new ProfiledPIDController(1.5, 0, .1, new Constraints(1000, 1000));
        tzPidController = new ProfiledPIDController(1.5, 0, .1, new Constraints(1000, 1000));
        yawPidController = new ProfiledPIDController(0.05, 0, 0, new Constraints(1000, 1000));
        this.swerveDrive = swerveDrive;
        this.io = io;
        setDefaultCommand(robotLocalization());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("FieldTracking", inputs);
    }
    public Command robotLocalization() {
        return run(() -> {
         
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
                swerveDrive.addVisionMeasurement(
                        inputs.pose,
                        inputs.timestampSeconds,
                        VecBuilder.fill(.7, .7, 9999999)
                        );
                
                Logger.recordOutput("FieldTracking/TargetPoses", new Pose3d[]{fieldLayout.getTagPose((int)inputs.tid).get()});
                Logger.recordOutput("FieldTracking/TargetIDs", new int[]{(int)inputs.tid});
            }
        }).ignoringDisable(true);
    }

    public boolean isAprilTagDetected() {
        // maybe convert to int?
        return inputs.tid != -1;
    }

    // public Command followAprilTag() {
    //     return run(() -> {
    //         if (!isAprilTagDetected()) {
    //             ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
    //             swerveDrive.updateSpeed(speeds); // this funciton sets everything to 0
    //             return;
    //         }
    
    public Command followAprilTag() {
        return run(() -> {
        if (!isAprilTagDetected()) {
            ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
            swerveDrive.updateSpeed(speeds); // this funciton sets everything to 0
            return;
        } else{   

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
            double xout = txPidController.calculate(-tx, 0);
            double zout = tzPidController.calculate(-tz, 1);
            double yawout = yawPidController.calculate(-yaw, 0);

            
            // step three take pid values and put it into swervedrive
            ChassisSpeeds speeds = new ChassisSpeeds(zout, xout, yawout);

            swerveDrive.updateSpeed(speeds); // this will update the speeed
            
        }
        })
        .until(this::isAtPosition);
    }
    public boolean isAtPosition(){
        return txPidController.atGoal() && tzPidController.atGoal() && yawPidController.atGoal();
    }
    
}
