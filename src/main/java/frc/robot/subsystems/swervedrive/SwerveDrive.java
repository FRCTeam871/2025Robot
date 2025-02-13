package frc.robot.subsystems.swervedrive;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerveModule.SwerveModule;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModule[] swerveModules;
    private final SwerveDriveKinematics swerveDriveKinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private SwerveDriveIO io;
    private SwerveDriveIOInputsAutoLogged inputs = new SwerveDriveIOInputsAutoLogged();
    private RobotConfig config;

    public SwerveDrive(SwerveDriveIO io,  final SwerveModule... swerveModules) {
        this.swerveModules = swerveModules;

        this.io = io;
        final Translation2d[] leverArmArray = Arrays.stream(swerveModules)
                .map(SwerveModule::getLeverArm)
                .toArray(Translation2d[]::new);

        this.swerveDriveKinematics = new SwerveDriveKinematics(leverArmArray);
        poseEstimator = new SwerveDrivePoseEstimator(swerveDriveKinematics, getRotation(), getModulePositions(), new Pose2d());         

        try {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                poseEstimator::getEstimatedPosition,
                this::resetOdometry,
                this::getChassisSpeeds,
                this::updateSpeed,
                new PPHolonomicDriveController(
                        new PIDConstants(1, 0, 0),
                        new PIDConstants(1, 0, 0),
                        4.5
                ),
                config,
                () -> false,
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("AutoBuilder.configure failed", e.getStackTrace());
        }
        
    }
    public RobotConfig getConfig(){
        return config;
    }

    public Command manualDrive(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omegarad) {
        return run(() -> {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vy.getAsDouble()*Constants.MAX_SPEED_MPS, vx.getAsDouble()*Constants.MAX_SPEED_MPS, omegarad.getAsDouble()*Constants.MAX_SPEED_MPS);
            updateSpeed(chassisSpeeds);
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);


        poseEstimator.update(getRotation(), getModulePositions());
        for (SwerveModule bob : swerveModules) {
            bob.periodic();
        }
    }

    @AutoLogOutput(key = "Drive/ChassisSpeeds")
    public ChassisSpeeds getChassisSpeeds() {
        return swerveDriveKinematics.toChassisSpeeds(
                getModuleStates());
    }

    @AutoLogOutput(key = "Drive/ModuleStates")
    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(swerveModules)
                .map(SwerveModule::getState)
                .toArray(SwerveModuleState[]::new);
    }

    @AutoLogOutput(key = "Drive/ModulePositions")
    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(swerveModules)
                .map(SwerveModule::getPosition)
                .toArray(SwerveModulePosition[]::new);
    }

    private void setStates(SwerveModuleState[] states) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setState(states[i]);
        }
    }
    /** 
     * +x is foward 
     * +y is left
     * + is ccw
     */
    public void updateSpeed(ChassisSpeeds speeds) {
        Logger.recordOutput("Drive/InputSpeed", speeds);
        // System.out.println(speeds);

        SwerveModuleState[] states = swerveDriveKinematics.toSwerveModuleStates(speeds);
        setStates(states);
    }

    public Command resetOdometryCommand() {
        final Command robert = runOnce(this::resetOdometry).ignoringDisable(true);
        robert.setName("Reset Odometry");
        return robert;
    }

    public Command resetOdometryCommand(Pose2d trajectory) {
        return runOnce(() -> resetOdometry(trajectory.getTranslation(), trajectory.getRotation()))
                .ignoringDisable(true);
    }

    public void resetOdometry() {
        resetOdometry(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
    }

    public void resetOdometry(Pose2d pose) {
        resetOdometry(pose.getTranslation(), pose.getRotation());
    }

    public void resetOdometry(Translation2d position, Rotation2d direction) {
        poseEstimator.resetPose(new Pose2d(position, direction));
        // swerveDriveOdometry.resetPosition(getRotation(), getModulePositions(), new Pose2d(position, direction));
    }

    @AutoLogOutput(key = "Drive/Rotation")
    public Rotation2d getRotation() {
        return inputs.gyroRotation;
    }

    @AutoLogOutput(key = "Drive/EstimatedPose")
    public Pose2d getEstimatedPose(){
       return poseEstimator.getEstimatedPosition();
    }
    
    public double getYawRate(){
        return inputs.gyroRate;
    }

    public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
      poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    

}
