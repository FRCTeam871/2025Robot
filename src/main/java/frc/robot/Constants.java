package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.swerveModule.SwerveModuleIO;
import frc.robot.subsystems.swerveModule.SwerveModuleIOSparkFlex;
import frc.robot.subsystems.swerveModule.SwerveModuleIOSparkMax;

public class Constants {
    public static final boolean ON_SYMPHONY = false;

    // Swerve Constants
    public static final double SWERVE_STEER_KP = 2 / 360.0;
    public static final double SWERVE_STEER_KI = 0.000;
    public static final double SWERVE_STEER_KD = 0;

    public static final double MAX_VELOCITY = 36000;
    public static final double MAX_ACCELERATION = 100000;

    public static final double DRIVE_SPEED_MULTIPLIER = 1.0;

    public static final double RADIUS_IN_METERS = edu.wpi.first.math.util.Units.inchesToMeters(1.85);
    public static final double SWERVE_DRIVE_RATIO = 1 / 6.75;
    public static final double SWERVE_POSITION_FACTOR = RADIUS_IN_METERS * 2 * Math.PI * SWERVE_DRIVE_RATIO;
    public static final double NEO_MAX_RPM = 5676;
    public static final double MAX_SPEED_MPS = (NEO_MAX_RPM / 60.0d) * SWERVE_POSITION_FACTOR;
    public static final double LEVER_ARM_VAL = (Constants.DISTANCE_BETWEEN_WHEELS / 39.37) / 2;

    public static final double DISTANCE_BETWEEN_WHEELS = 22.75;
    // INTAKE CONSTANTS
    public static final Time PISTON_THRESHOLD = Units.Second.of(1.0);
    public static final Time TARGET_DROP_THRESHOLD = Units.Second.of(0.5);
    public static final Time PISTON_OUT_TIME = Units.Second.of(0.25);

    public static final double BOTTOM = 1; 
    public static final double L1 = 5;
    public static final double L2 = 10;
    public static final double L3 = 25;
    public static final double L4 = 50; 
    public static final double CLIMBING_MOUNT = 60;

    public record ModuleConstants(
            String label,
            int driveId,
            boolean driveInverted,
            int steerId,
            boolean steerInverted,
            int encoderId,
            double encoderOffset,
            SensorDirectionValue encoderDirection,
            int dashBoardX,
            int dashBoardY,
            Translation2d leverArm) {
    }

    //positive x is foward and positive y is left
    public static final ModuleConstants[] MODULE_CONSTANTS = new ModuleConstants[] {
        new ModuleConstants(
                "FL",
                9,
                false,
                7,
                true,
                8,
                -0.360840,
                SensorDirectionValue.CounterClockwise_Positive,
                3,
                0,
                new Translation2d(Constants.LEVER_ARM_VAL, Constants.LEVER_ARM_VAL)),
        new ModuleConstants(
                "FR",
                12,
                false,
                10,
                true,
                11,
                -0.555176,
                SensorDirectionValue.CounterClockwise_Positive,
                0,
                0,
                new Translation2d(Constants.LEVER_ARM_VAL, -Constants.LEVER_ARM_VAL)),
        new ModuleConstants(
                "BL",
                6,
                false,
                4,
                true,
                5,
                0.613037,
                SensorDirectionValue.CounterClockwise_Positive,
                0,
                0,
                new Translation2d(-Constants.LEVER_ARM_VAL, Constants.LEVER_ARM_VAL)),
        new ModuleConstants(
                "BR",
                3,
                false,
                1,
                true,
                2,
                -0.481445,
                SensorDirectionValue.CounterClockwise_Positive,
                3,
                5,
                new Translation2d(-Constants.LEVER_ARM_VAL, -Constants.LEVER_ARM_VAL))
};

    //positive x is foward and positive y is left
    public static final ModuleConstants[] MODULE_CONSTANTS_SYMPHONY = new ModuleConstants[] {
        new ModuleConstants(
                "FL",
                3,
                false,
                1,
                true,
                2,
                -0.0185555,
                SensorDirectionValue.CounterClockwise_Positive,
                3,
                0,
                new Translation2d(Constants.LEVER_ARM_VAL, Constants.LEVER_ARM_VAL)),
        new ModuleConstants(
                "FR",
                4,
                true,
                6,
                true,
                5,
                -0.060059,
                SensorDirectionValue.CounterClockwise_Positive,
                0,
                0,
                new Translation2d(Constants.LEVER_ARM_VAL, -Constants.LEVER_ARM_VAL)),
        new ModuleConstants(
                "BL",
                10,
                true,
                12,
                true,
                11,
                -0.0434570,
                SensorDirectionValue.CounterClockwise_Positive,
                0,
                0,
                new Translation2d(-Constants.LEVER_ARM_VAL, Constants.LEVER_ARM_VAL)),
        new ModuleConstants(
                "BR",
                9,
                true,
                7,
                true,
                8,
                -0.019287109375,
                SensorDirectionValue.CounterClockwise_Positive,
                3,
                5,
                new Translation2d(-Constants.LEVER_ARM_VAL, -Constants.LEVER_ARM_VAL))
    };

    public static SwerveModuleIO getRealSwerveModuleIO(ModuleConstants moduleConstants) {
        if (ON_SYMPHONY) {
            return new SwerveModuleIOSparkMax(moduleConstants);
        } else {
            return new SwerveModuleIOSparkFlex(moduleConstants);
        }

    }

    public static double deadband(double raw, double threshold) {

        if (Math.abs(raw) < threshold) {
            return 0;
        } else {
            if (raw > 0) {
                return (raw - threshold) / (1 - threshold);
            } else {
                return (raw + threshold) / (1 - threshold);
            }
        }

    }

    public static double deadbandAndExponential(double raw) {
        return exponentialDrive(deadband(raw, 0.1));
    }

    public static double exponentialDrive(double controllerOutput) {
        double contollerOutputA = 15;
        double controllerOutputB = 0.015;
        double controllerOutputC = (1 - controllerOutputB) / (contollerOutputA - 1);
        double wrappedControllerOutput = controllerOutputC * Math.pow(contollerOutputA, Math.abs(controllerOutput))
                + controllerOutputB * Math.abs(controllerOutput)
                - controllerOutputC;
        if (controllerOutput >= 0) {
            return wrappedControllerOutput;
        } else {
            return -wrappedControllerOutput;
        }
    }

}
