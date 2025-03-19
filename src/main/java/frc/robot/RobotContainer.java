// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.Collections;
import java.util.stream.IntStream;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ModuleConstants;
import frc.robot.controls.IControls;
import frc.robot.controls.XboxControls;
import frc.robot.subsystems.swerveModule.SwerveModule;
import frc.robot.subsystems.swerveModule.SwerveModuleIO;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveDriveIO;
import frc.robot.subsystems.swervedrive.SwerveDriveIOYaw;

public class RobotContainer {
    final SwerveDrive swerveDrive;
    final IControls controls;

    public RobotContainer() {
        this.controls = new XboxControls();
        SwerveModuleIO[] moduleIOs = Collections.nCopies(4, SwerveModuleIO.EMPTY).toArray(SwerveModuleIO[]::new);
        SwerveDriveIO swerveDriveIO = SwerveDriveIO.EMPTY;

        if (RobotBase.isReal()) {
            moduleIOs = Arrays.stream(Constants.MODULE_CONSTANTS)
                    .map(Constants::getRealSwerveModuleIO)
                    .toArray(SwerveModuleIO[]::new);

            swerveDriveIO = new SwerveDriveIOYaw(new AHRS(NavXComType.kMXP_SPI));
        }

        final SwerveModuleIO[] moduleIOsFinal = moduleIOs;
        final SwerveModule[] swerveModules = IntStream.range(0, moduleIOs.length)
                .mapToObj(i -> {
                    final SwerveModuleIO io = moduleIOsFinal[i];
                    final ModuleConstants constants = Constants.MODULE_CONSTANTS[i];
                    return new SwerveModule(constants.leverArm(), io, constants.label());
                })
                .toArray(SwerveModule[]::new);

        swerveDrive = new SwerveDrive(swerveDriveIO, swerveModules);
        configureBindings();
    }

    private void configureBindings() {
        swerveDrive.setDefaultCommand(swerveDrive.manualDrive(
                controls.sideToSideAxis(), controls.fowardsAndBackAxis(), controls.driveRotation()));
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    public Command getTeleopCommand() {
        return Commands.none();
    }

    public void disabledInit() {

    }

    public void autonomousInit() {

    }

    public void teleopInit() {

    }

    public void disabledPeriodic() {

    }

    public void robotPeriodic() {

    }

    public boolean isReadyForMatch() {
        return true;
    }
}
