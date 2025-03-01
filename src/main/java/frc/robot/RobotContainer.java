// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

    public RobotContainer() {
        this.controls = new XboxControls();
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        SwerveModuleIO[] moduleIOs = Collections.nCopies(4, SwerveModuleIO.EMPTY).toArray(SwerveModuleIO[]::new);
        ;
        SwerveDriveIO swerveDriveIO = SwerveDriveIO.EMPTY;
        ElevatorIO elevatorIO = ElevatorIO.EMPTY;
        SequencingIO sequencingIO = SequencingIO.EMPTY;

        ManipulatorIO manipulatorIO = ManipulatorIO.EMPTY;
        ;
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
            }
            // fieldTrackingIO = new FieldTrackingIOLimeLight();
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
        intake = new Intake(intakeIO);
        manipulator = new Manipulator(manipulatorIO, fieldTracking);
        sequencing = new Sequencing(elevator, intake, swerveDrive, manipulator, fieldTracking, sequencingIO);
        led = new LEDs(ledio);
        configureBindings();
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

        // sequencing.bindScoreCoral(controls.placeCoral());
        // controls.cancel().onTrue(Commands.runOnce(()->
        // sequencing.cancelScoreCoral()));

        // elevator.setDefaultCommand(elevator.manualControl(controls.elevatorMove()));
        controls.switchManualElevator().toggleOnTrue(elevator.manualControl(controls.elevatorMove()));
        // controls.goToNearestAprilTag().whileTrue(fieldTracking.followAprilTag());
        // controls.goToNearestAprilTag().whileTrue(intake.sendLeftPistonOut());

        controls.pushCoral().onTrue((new ConditionalCommand(Commands.run(()->{}),manipulator.pushCoral(), ()-> elevator.getSetPoint() == Setpoint.L4)).beforeStarting(Commands.run(() -> {
        }).withTimeout(.02)).alongWith(manipulator.releaseCoral()).withTimeout(.8).finallyDo(() -> elevator.goToSetpoint(Setpoint.Bottom).schedule()));
        // controls.releaseCoral().whileTrue(manipulator.releaseCoral().andThen(elevator.goToSetpoint(Setpoint.Bottom)));
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

    public Command getAutonomousCommand() {
        return null;
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
        fieldTracking.setCameraIMUMode(IMUMode.ExternalReset); // TODO: use InternalMT1Assist when LL update comes out
        fieldTracking.setThrottle(6);
    }

    public void autonomousInit() {
        fieldTracking.setCameraIMUMode(IMUMode.InternalMT1Assist);
        fieldTracking.setThrottle(0);
    }

    public void teleopInit() {
        fieldTracking.setCameraIMUMode(IMUMode.InternalMT1Assist);
        fieldTracking.setThrottle(0);
    }
}
