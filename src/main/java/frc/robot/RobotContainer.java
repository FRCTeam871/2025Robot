// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.AutonomousPlanner;
import frc.robot.Constants.ModuleConstants;
import frc.robot.controls.IControls;
import frc.robot.controls.XboxControls;
import frc.robot.subsystems.LEDSubsystem.LEDIO;
import frc.robot.subsystems.LEDSubsystem.LEDs;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.Elevator.Setpoint;
import frc.robot.subsystems.fieldtracking.FieldTracking;
import frc.robot.subsystems.fieldtracking.FieldTrackingIO;
import frc.robot.subsystems.fieldtracking.FieldTrackingIO.IMUMode;
import frc.robot.subsystems.fieldtracking.FieldTrackingIOLimeLight;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOReal;
import frc.robot.subsystems.sequencing.Sequencing;
import frc.robot.subsystems.sequencing.Sequencing.LeftOrRight;
import frc.robot.subsystems.sequencing.Sequencing.ReefLevel;
import frc.robot.subsystems.sequencing.Sequencing.ReefSides;
import frc.robot.subsystems.sequencing.SequencingIO;
import frc.robot.subsystems.swerveModule.SwerveModule;
import frc.robot.subsystems.swerveModule.SwerveModuleIO;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveDriveIO;
import frc.robot.subsystems.swervedrive.SwerveDriveIORoll;
import frc.robot.subsystems.swervedrive.SwerveDriveIOYaw;
import frc.robot.subsystems.zoneOperator.ZoneOperator;

import java.util.Arrays;
import java.util.Collections;
import java.util.stream.IntStream;

public class RobotContainer {
    final SwerveDrive swerveDrive;
    final FieldTracking fieldTracking;
    final IControls controls;
    final Intake intake;
    final Elevator elevator;
    final Sequencing sequencing;
    final Manipulator manipulator;
    final LEDs led;
    final Compressor compressor;
    final ZoneOperator zoneOperator;
    final AutonomousPlanner autonomousPlanner;
    Elevator.Setpoint storedLevel;

    public RobotContainer() {
        storedLevel = Elevator.Setpoint.L1;
        this.controls = new XboxControls();
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        SwerveModuleIO[] moduleIOs = Collections.nCopies(4, SwerveModuleIO.EMPTY).toArray(SwerveModuleIO[]::new);
        SwerveDriveIO swerveDriveIO = SwerveDriveIO.EMPTY;
        ElevatorIO elevatorIO = ElevatorIO.EMPTY;
        SequencingIO sequencingIO = SequencingIO.EMPTY;

        ManipulatorIO manipulatorIO = ManipulatorIO.EMPTY;
        
        IntakeIO intakeIO = IntakeIO.EMPTY;
        FieldTrackingIO fieldTrackingIO = FieldTrackingIO.EMPTY;
        LEDIO ledio = LEDIO.EMPTY;
        if (RobotBase.isReal()) {
            moduleIOs = Arrays.stream(
                    Constants.ON_SYMPHONY ? Constants.MODULE_CONSTANTS_SYMPHONY : Constants.MODULE_CONSTANTS)
                    .map(Constants::getRealSwerveModuleIO)
                    .toArray(SwerveModuleIO[]::new);

            if (Constants.ON_SYMPHONY) {
                swerveDriveIO = new SwerveDriveIOYaw(new AHRS(NavXComType.kMXP_SPI));
            } else {
                swerveDriveIO = new SwerveDriveIORoll(new AHRS(NavXComType.kMXP_SPI));
                elevatorIO = new ElevatorIOReal();
                intakeIO = new IntakeIOReal();
                manipulatorIO = new ManipulatorIOReal();
                fieldTrackingIO = new FieldTrackingIOLimeLight();
            }
            // ledio = new LEDIOReal();
        }

        final SwerveModuleIO[] moduleIOsFinal = moduleIOs;
        final SwerveModule[] swerveModules = IntStream.range(0, moduleIOs.length)
                .mapToObj(i -> {
                    final SwerveModuleIO io = moduleIOsFinal[i];
                    final ModuleConstants constants = Constants.MODULE_CONSTANTS[i];
                    return new SwerveModule(constants.leverArm(), io, constants.label());
                })
                .toArray(SwerveModule[]::new);

        elevator = new Elevator(elevatorIO);
        swerveDrive = new SwerveDrive(swerveDriveIO, elevator, swerveModules);
        fieldTracking = new FieldTracking(swerveDrive, fieldTrackingIO);
        intake = new Intake(intakeIO, elevator);
        manipulator = new Manipulator(manipulatorIO, fieldTracking);
        sequencing = new Sequencing(elevator, intake, swerveDrive, manipulator, fieldTracking, sequencingIO);
        zoneOperator = new ZoneOperator(swerveDrive);
        autonomousPlanner = new AutonomousPlanner(elevator, intake, manipulator, swerveDrive, sequencing, fieldTracking);
        led = new LEDs(ledio);
        configureBindings();
        configureZones();
    }

    private void configureBindings() {
        swerveDrive.setDefaultCommand(swerveDrive.manualDrive(
                controls.sideToSideAxis(), controls.fowardsAndBackAxis(), controls.driveRotation()));

        controls.manualElevatorMoveDown()
                .onTrue(elevator.goToSetpoint(() -> {
                    System.out.println(elevator.getSetPoint().nextDown());
                    System.out.println(elevator.getSetPoint());
                    return elevator.getSetPoint().nextDown();
                })
                        .ignoringDisable(true));
        controls.manualElevatorMoveUp()
                .onTrue(elevator.goToSetpoint(() -> {
                    System.out.println(elevator.getSetPoint().nextUp());
                    System.out.println(elevator.getSetPoint());
                    return elevator.getSetPoint().nextUp();
                })
                        .ignoringDisable(true));

        controls.placeCoral()
                .onTrue(sequencing
                        .scoreCoral(ReefSides.Side1, LeftOrRight.Right, ReefLevel.L4)
                        .until(() -> controls.cancel().getAsBoolean()));

        controls.fieldOrientationToggle().onTrue(Commands.runOnce(()-> {
            if(swerveDrive.isFieldOrientation()){
                swerveDrive.setFieldOrientation(false);
            } else{
                swerveDrive.setFieldOrientation(true);
            }
        }));

        controls.buttonL1().onTrue(Commands.runOnce(()-> storedLevel = Elevator.Setpoint.L3));

        // sequencing.bindScoreCoral(controls.placeCoral());
        // controls.cancel().onTrue(Commands.runOnce(()->
        // sequencing.cancelScoreCoral()));

        // elevator.setDefaultCommand(elevator.manualControl(controls.elevatorMove()));
        controls.switchManualElevator().toggleOnTrue(elevator.manualControl(controls.elevatorMove()));
        // controls.goToNearestAprilTag().whileTrue(fieldTracking.followAprilTag());
        // controls.goToNearestAprilTag().whileTrue(intake.sendLeftPistonOut());

        controls.pushCoral().onTrue((new ConditionalCommand(Commands.run(()->{}),manipulator.pushCoral(), ()-> elevator.getSetPoint() == Setpoint.L4)).beforeStarting(Commands.run(() -> {
        }).withTimeout(.02)).alongWith(manipulator.releaseCoral()).withTimeout(.8).finallyDo(() -> elevator.goToSetpoint(Setpoint.Bottom).schedule()));
        controls.releaseCoral().whileTrue(manipulator.releaseCoral());
        // controls.pushCoral().whileTrue(manipulator.pushCoral());

        controls.intakePiston1().whileTrue(intake.sendLeftPistonOut());
        controls.intakePiston2().whileTrue(intake.sendRightPistonOut());

        controls.compressorToggle().onTrue(
                Commands.runOnce(() -> {
                    if (compressor.isEnabled()) {
                        compressor.disable();      
                    } else {
                        compressor.enableDigital();
                    }
                }));
    }

    private void configureZones() {
        zoneOperator.addCircle(
            new Translation2d(4.483, 3.995), 
            Units.Meters.of(2), 
            Commands.runOnce(()-> elevator.goToSetpoint(storedLevel).schedule()).andThen(Commands.run(()->{})).finallyDo(()-> elevator.goToSetpoint(Setpoint.Bottom).schedule()),
            "ReefZone"
        );
        zoneOperator.addCircle(
            new Translation2d(1.103, 6.944), 
            Units.Meters.of(2),
            swerveDrive.doHeadingHoldBlueRelative(Rotation2d.fromDegrees(-55)),
            "CoralStationLeft"
        );
        zoneOperator.addCircle(
            new Translation2d(3.194, 4.205), 
            Units.Meters.of(.3),
            swerveDrive.doPoseHoldBlueRelative(new Pose2d(3.194, 4.205, Rotation2d.kZero)),
            "ReefLeft"
        );
        zoneOperator.addPolygon(new Translation2d[]{
            new Translation2d(3.625,4.514),
            new Translation2d(3.625,3.550),            
            new Translation2d(2.805,3.097),
            new Translation2d(2.805,4.946),

        }, swerveDrive.doHeadingHoldBlueRelative(Rotation2d.fromDegrees(0)), "gregory" ); // ??

        // for (ReefSides side : ReefSides.values()) {
        //     Pose2d leftPose = sequencing.reefPose(side, LeftOrRight.Left); 
        //     Pose2d rightPose = sequencing.reefPose(side, LeftOrRight.Right); 
        //     Pose2d centerPose = sequencing.reefPose(side);
            
        //     zoneOperator.addCircle(
        //         leftPose.getTranslation(), 
        //         Units.Meters.of(1),
        //         swerveDrive.doPoseHoldBlueRelative(leftPose),
        //         "CoralStationLeft"
        //     );
        //     zoneOperator.addCircle(
        //         rightPose.getTranslation(), 
        //         Units.Meters.of(1),
        //         swerveDrive.doPoseHoldBlueRelative(rightPose),
        //         "CoralStationRight"
        //     );
        // }
    }

    public Command getAutonomousCommand() {
        return autonomousPlanner.getAutonCommand();
    }

    public Command getTeleopCommand() {
        return Commands.none();
        // return fieldTracking.followAprilTag();
        // // // return Commands.run(() -> {
        // // // ChassisSpeeds swerveSpeeds = new ChassisSpeeds(.1, 0, 0);
        // // // swerveDrive.updateSpeed(swerveSpeeds);

        // // // }, swerveDrive);

    }

    public void disabledInit() {
        // fieldTracking.setCameraIMUMode(IMUMode.InternalMT1Assist);
        fieldTracking.setCameraIMUMode(IMUMode.InternalExternalAssist);
        fieldTracking.setThrottle(6);
        fieldTracking.setIMUAssistAlpha(.1);
        zoneOperator.setEnabled(false);
    }

    public void autonomousInit() {
        fieldTracking.setCameraIMUMode(IMUMode.InternalExternalAssist);
        fieldTracking.setThrottle(0);
        zoneOperator.setEnabled(false);
    }

    public void teleopInit() {
        fieldTracking.setCameraIMUMode(IMUMode.InternalExternalAssist);
        fieldTracking.setThrottle(0);
        fieldTracking.setIMUAssistAlpha(.005);
        zoneOperator.setEnabled(true);
    }
    public void disabledPeriodic() {
        Pose2d cameraPose = fieldTracking.getLimeLightPose();
        if(fieldTracking.isAprilTagDetected()){
        // swerveDrive.setCurrentAngle(cameraPose.getRotation().getDegrees());
        }
    }
}
