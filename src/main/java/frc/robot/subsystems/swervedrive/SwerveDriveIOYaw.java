package frc.robot.subsystems.swervedrive;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

public class SwerveDriveIOYaw implements SwerveDriveIO {
    AHRS gyro;

    public SwerveDriveIOYaw(AHRS gyro) {
        SmartDashboard.putData("resetGyro", Commands.runOnce(gyro::zeroYaw).ignoringDisable(true));
        gyro.zeroYaw();
        this.gyro = gyro;
    }

    @Override
    public void updateInputs(SwerveDriveIOInputs inputs) {
        inputs.gyroRotation = Rotation2d.fromDegrees(-gyro.getYaw() + 120);
        inputs.isCalibrating = gyro.isCalibrating();
        inputs.gyroRate = gyro.getRate();
    }
}
