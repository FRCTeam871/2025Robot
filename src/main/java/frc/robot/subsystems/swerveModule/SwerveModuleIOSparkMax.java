package frc.robot.subsystems.swerveModule;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private final SparkMax drive;
    private final SparkMax steer;
    private final CANcoder steeringEncoder;
    private final RelativeEncoder distanceEncoder;

    public SwerveModuleIOSparkMax(
        final ModuleConstants moduleConstants
            ) {
        this.drive = new SparkMax(moduleConstants.driveId(), MotorType.kBrushless);
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.inverted(!moduleConstants.driveInverted());
        driveConfig.encoder.positionConversionFactor(Constants.SWERVE_POSITION_FACTOR);
        driveConfig.encoder.velocityConversionFactor(Constants.SWERVE_POSITION_FACTOR / 60);
        this.drive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.distanceEncoder = drive.getEncoder();

        this.steer = new SparkMax(moduleConstants.steerId(), MotorType.kBrushless);
        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig.idleMode(IdleMode.kBrake);
        steerConfig.inverted(moduleConstants.steerInverted());
        this.steer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        final CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = moduleConstants.encoderOffset();
        config.MagnetSensor.SensorDirection = moduleConstants.encoderDirection();
        this.steeringEncoder = new CANcoder(moduleConstants.encoderId());
        this.steeringEncoder.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePosition = distanceEncoder.getPosition();
        inputs.driveVelocity = distanceEncoder.getVelocity();
        inputs.steeringAngleDegrees = steeringEncoder.getAbsolutePosition().getValue().in(Units.Degrees);
    }

    @Override
    public void setDriveSpeed(double speed){
        drive.set(speed*Constants.DRIVE_SPEED_MULTIPLIER);
    }

    @Override
    public void setSteerSpeed(double speed){
        steer.set(speed);

    }
}
