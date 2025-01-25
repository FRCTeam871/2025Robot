// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ModuleConstants;
import frc.robot.controls.IControls;
import frc.robot.controls.XboxControls;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorIO;
import frc.robot.subsystems.ElevatorIOReal;
import frc.robot.subsystems.FieldTracking;
import frc.robot.subsystems.FieldTrackingIO;
import frc.robot.subsystems.FieldTrackingIOLimeLight;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeIO;
import frc.robot.subsystems.IntakeIOReal;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDriveIO;
import frc.robot.subsystems.SwerveDriveIOReal;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveModuleIO;
import frc.robot.subsystems.SwerveModuleIOSparkFlex;

import java.util.Arrays;
import java.util.Collections;
import java.util.Map;
import java.util.stream.IntStream;
import java.util.stream.StreamSupport;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class RobotContainer {
        SwerveDrive swerveDrive;
        FieldTracking fieldTracking;
        IControls controls;
        Intake intake;
        Elevator elevator;


        public RobotContainer() {
                
                controls = new XboxControls();
                LiveWindow.disableAllTelemetry();
                // Configure the trigger bindings

                SwerveModuleIO[] moduleIOs;
                SwerveDriveIO swerveDriveIO;
                IntakeIO intakeIO;
                ElevatorIO elevatorIO;

                if (RobotBase.isReal()) {

                        swerveDriveIO = new SwerveDriveIOReal(new AHRS(NavXComType.kI2C));

                        moduleIOs = Arrays.stream(Constants.MODULE_CONSTANTS)
                                        .map(Constants::getRealSwerveModuleIO)
                                        .toArray(SwerveModuleIO[]::new);
                        
                        intakeIO = new IntakeIOReal();

                        elevatorIO = new ElevatorIOReal();

                } else {
                        moduleIOs = Collections.nCopies(4, new SwerveModuleIO() {
                        }).toArray(new SwerveModuleIO[4]);
                        swerveDriveIO = new SwerveDriveIO() {
                        };

                        intakeIO = new IntakeIO() {};

                        elevatorIO = new ElevatorIO() {};
                }

                ShuffleboardTab swerveShuffleboardTab = Shuffleboard.getTab("Symphony");

                SwerveModule[] swerveModules = IntStream.range(0, moduleIOs.length)
                                .mapToObj(i -> {
                                        SwerveModuleIO io = moduleIOs[i];
                                        ModuleConstants constants = Constants.MODULE_CONSTANTS[i];

                                        ShuffleboardLayout dashboardLayOut = swerveShuffleboardTab
                                                        .getLayout(constants.label(), BuiltInLayouts.kGrid)
                                                        .withSize(3, 5)
                                                        .withProperties(Map.of("Number of columns", 1, "Number of rows",
                                                                        3))
                                                        .withPosition(constants.dashBoardX(), constants.dashBoardY());

                                        final SwerveModule swerve = new SwerveModule(
                                                        constants.leverArm(),
                                                        dashboardLayOut, io, constants.label());
                                        return swerve;
                                })
                                .toArray(SwerveModule[]::new);

              

                ShuffleboardLayout odometryLayout = swerveShuffleboardTab.getLayout("Odometry", BuiltInLayouts.kGrid)
                                .withSize(5, 8)
                                .withProperties(Map.of("Number of columns", 2, "Number of rows", 3))
                                .withPosition(13, 0);
                swerveDrive = new SwerveDrive(swerveDriveIO,
                                odometryLayout,
                                swerveModules);
                swerveShuffleboardTab.addCamera("Title", "name", "mjpg:http://10.8.71.86:5800")
                                .withPosition(6, 0)
                                .withSize(7, 8)
                                .withProperties(Map.of("showControls", false));

                if (RobotBase.isReal()) {
                        fieldTracking = new FieldTracking(swerveDrive, new FieldTrackingIOLimeLight());
                } else {
                        fieldTracking = new FieldTracking(swerveDrive, new FieldTrackingIO() {
                        });
                }

                intake = new Intake(intakeIO);




                configureBindings();
        }

        private void configureBindings() {
                controls.goToNearestAprilTag().whileTrue(fieldTracking.followAprilTag());

                // controls.goToNearestAprilTag().whileTrue(intake.sendLeftPistonOut());
                
                controls.elevatorMoveToSetPoint().onTrue(elevator.goToSetPoint(Constants.ELEVATOR_SETPOINT));
        }

        public Command getAutonomousCommand() {
                return null;
        }

        public Command getTeleopCommand() {
               return fieldTracking.followAprilTag();
        // //         // return Commands.run(() -> {
        // //         // ChassisSpeeds swerveSpeeds = new ChassisSpeeds(.1, 0, 0);
        // //         // swerveDrive.updateSpeed(swerveSpeeds);

        // //         // }, swerveDrive);

        }

}
