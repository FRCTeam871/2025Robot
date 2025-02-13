// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ModuleConstants;
import frc.robot.controls.IControls;
import frc.robot.controls.XboxControls;
import frc.robot.subsystems.LEDSubsystem.LEDIO;
import frc.robot.subsystems.LEDSubsystem.LEDs;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.fieldtracking.FieldTracking;
import frc.robot.subsystems.fieldtracking.FieldTrackingIO;
import frc.robot.subsystems.fieldtracking.FieldTrackingIO.IMUMode;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.sequencing.Sequencing;
import frc.robot.subsystems.sequencing.SequencingIO;
import frc.robot.subsystems.sequencing.SequencingIOReal;
import frc.robot.subsystems.sequencing.Sequencing.LeftOrRight;
import frc.robot.subsystems.sequencing.Sequencing.ReefLevel;
import frc.robot.subsystems.sequencing.Sequencing.ReefSides;
import frc.robot.subsystems.swerveModule.SwerveModule;
import frc.robot.subsystems.swerveModule.SwerveModuleIO;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveDriveIO;
import frc.robot.subsystems.swervedrive.SwerveDriveIORoll;
import frc.robot.subsystems.swervedrive.SwerveDriveIOYaw;

import java.util.Arrays;
import java.util.Collections;
import java.util.stream.IntStream;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class RobotContainer {
        SwerveDrive swerveDrive;
        FieldTracking fieldTracking;
        IControls controls;
        Intake intake;
        Elevator elevator;
        Sequencing sequencing;
        Manipulator manipulator;
        LEDs led;
        
        
        public RobotContainer() {
                
                controls = new XboxControls();
                LiveWindow.disableAllTelemetry();
                // Configure the trigger bindings

                SwerveModuleIO[] moduleIOs = Collections.nCopies(4, new SwerveModuleIO() {
                }).toArray(new SwerveModuleIO[4]);
                SwerveDriveIO swerveDriveIO = new SwerveDriveIO() {};
                IntakeIO intakeIO = new IntakeIO() {};
                ElevatorIO elevatorIO = new ElevatorIO() {};
                ManipulatorIO   manipulatorIO = new ManipulatorIO() {};
                SequencingIO sequencingIO = new SequencingIO() {};
                FieldTrackingIO fieldTrackingIO = new FieldTrackingIO() {};
                LEDIO ledio = new LEDIO() {};

                
                if (RobotBase.isReal()) {

                        // fieldTrackingIO = new FieldTrackingIOLimeLight();

                        moduleIOs = Arrays.stream(Constants.ON_SYMPHONY ? Constants.MODULE_CONSTANTS_SYMPHONY : Constants.MODULE_CONSTANTS)
                                        .map(Constants::getRealSwerveModuleIO)
                                        .toArray(SwerveModuleIO[]::new);

                        sequencingIO = new SequencingIOReal() {};
                        // ledio = new LEDIOReal();

                        if (Constants.ON_SYMPHONY){
                                swerveDriveIO = new SwerveDriveIOYaw(new AHRS(NavXComType.kMXP_SPI));
                        } else {
                                swerveDriveIO = new SwerveDriveIORoll(new AHRS(NavXComType.kMXP_SPI));
                                // intakeIO = new IntakeIOReal();
                                elevatorIO = new ElevatorIOReal();
                                // manipulatorIO = new ManipulatorIOReal();
                        }                       
                }
                        
                final SwerveModuleIO[] moduleIOsFinal = moduleIOs;
                SwerveModule[] swerveModules = IntStream.range(0, moduleIOs.length)
                                .mapToObj(i -> {
                                        SwerveModuleIO io = moduleIOsFinal[i];
                                        ModuleConstants constants = Constants.MODULE_CONSTANTS[i];

                                        final SwerveModule swerve = new SwerveModule(
                                                        constants.leverArm(),
                                                        io, constants.label());
                                        return swerve;
                                })
                                .toArray(SwerveModule[]::new);

                

                
                swerveDrive = new SwerveDrive(swerveDriveIO,
                                swerveModules);
                fieldTracking = new FieldTracking(swerveDrive, fieldTrackingIO);
                intake = new Intake(intakeIO);
                manipulator = new Manipulator(manipulatorIO, fieldTracking);
                elevator = new Elevator(elevatorIO);
                sequencing = new Sequencing(elevator, intake, swerveDrive, manipulator, fieldTracking, sequencingIO);
                led = new LEDs(ledio);

                configureBindings();
        }

        private void configureBindings() {
                swerveDrive.setDefaultCommand(swerveDrive.manualDrive(controls.sideToSideAxis() ,controls.fowardsAndBackAxis(), controls.driveRotation()));
                // controls.goToNearestAprilTag().whileTrue(fieldTracking.followAprilTag());

                // controls.goToNearestAprilTag().whileTrue(intake.sendLeftPistonOut());
                // controls.manualElevatorMoveDown().onTrue(elevator.goToSetpoint(() -> elevator.getSetPoint().nextDown()));
                // controls.manualElevatorMoveUp().onTrue(elevator.goToSetpoint(() -> elevator.getSetPoint().nextUp()));

                elevator.setDefaultCommand(elevator.manualControl(controls.elevatorMove()));
                
                controls.placeCoral().onTrue(sequencing.scoreCoral(ReefSides.Side2, LeftOrRight.Right, ReefLevel.L4)
                        .until(() -> controls.cancel().getAsBoolean())); 
                
                
        }

        public Command getAutonomousCommand() {
                return null;
        }

        public Command getTeleopCommand() {
                return  Commands.none();
        //        return fieldTracking.followAprilTag();
        // //         // return Commands.run(() -> {
        // //         // ChassisSpeeds swerveSpeeds = new ChassisSpeeds(.1, 0, 0);
        // //         // swerveDrive.updateSpeed(swerveSpeeds);

        // //         // }, swerveDrive);

        }

        public void disabledInit(){
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
