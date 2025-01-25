package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants;

public class SwerveModule {
    private final ProfiledPIDController pidController;
    private final Translation2d leverArm;
    private SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(0));
    private SwerveModuleIO io; 
    // private final GenericEntry steerOutputEntry;
    // private final GenericEntry driveOutputEntry;
    SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private String label;
    // private final GenericEntry targetAngleEntry;


    public SwerveModule(
                        final Translation2d leverArm,
                        ShuffleboardLayout layout,
                        SwerveModuleIO io,
                        String label
                        ) {
        this.leverArm = leverArm;
        this.io = io;
        this.label = label;                            


        this.pidController = new ProfiledPIDController(Constants.SWERVE_STEER_KP,
                Constants.SWERVE_STEER_KI,
                Constants.SWERVE_STEER_KD,
                new TrapezoidProfile.Constraints(Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION));
        this.pidController.enableContinuousInput(0, 360);

        // Add stuff to the layout
        // this.steerOutputEntry = layout.add("Steer Power", 0)
        //         .withPosition(0, 0).getEntry();
        // layout.addDouble("Current Angle", this::getDirectionDegrees)
        //         .withPosition(0, 2)
        //         .withWidget(BuiltInWidgets.kGyro);
        // this.driveOutputEntry = layout.add("Drive Power", 0)
        //         .withPosition(0, 1)
        //         .getEntry();
        // this.targetAngleEntry = layout.add("Target Angle", 0.0d)
        //         .withPosition(1, 1)
        //         .withWidget(BuiltInWidgets.kGyro)
        //         .getEntry();
    }
    

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + label, inputs);

        final double currentSteeringAngle = getDirectionDegrees();
        final SwerveModuleState optimizedState = SwerveModuleState.optimize(state,
                Rotation2d.fromDegrees(currentSteeringAngle));
        final double desiredSteeringAngle = optimizedState.angle.getDegrees();
        final double outputSteer = pidController.calculate(currentSteeringAngle, desiredSteeringAngle);

        io.setDriveSpeed(optimizedState.speedMetersPerSecond/Constants.MAX_SPEED_MPS);
        Logger.recordOutput("Drive/Module" + label + "/DriveSpeed", optimizedState.speedMetersPerSecond/Constants.MAX_SPEED_MPS);
        io.setSteerSpeed(outputSteer);
        Logger.recordOutput("Steer/Module" + label + "/SteerSpeed", outputSteer);

        // targetAngleEntry.setDouble(desiredSteeringAngle);
        // driveOutputEntry.setDouble(optimizedState.speedMetersPerSecond/Constants.MAX_SPEED_MPS);
        // steerOutputEntry.setDouble(outputSteer);
    }

    public void setState(final SwerveModuleState state) {
        this.state = state;
    }

    @AutoLogOutput(key = "Drive/Module{label}/State")
    public SwerveModuleState getState() {
        return new SwerveModuleState(inputs.driveVelocity, Rotation2d.fromDegrees(getDirectionDegrees()));
    }
    
    @AutoLogOutput(key = "Drive/Module{label}/Position")
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePosition,
                Rotation2d.fromDegrees(getDirectionDegrees()));
    }

    private double getDirectionDegrees() {
        return inputs.steeringAngleDegrees;
    }

    public Translation2d getLeverArm() {
        return leverArm;
    }
}

