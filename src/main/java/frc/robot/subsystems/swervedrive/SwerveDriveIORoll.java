package frc.robot.subsystems.swervedrive;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;;

public class SwerveDriveIORoll implements SwerveDriveIO {
    AHRS gyro;
    double gyroZero;

    public SwerveDriveIORoll(AHRS gyro) {
        SmartDashboard.putData("resetGyro", Commands.runOnce(this::zeroRoll).ignoringDisable(true));
        this.gyro = gyro;
        zeroRoll();
    }

    @Override
    public void updateInputs(SwerveDriveIOInputs inputs) {
        inputs.gyroRotation = Rotation2d.fromDegrees(gyro.getRoll() - gyroZero);
        inputs.isCalibrating = gyro.isCalibrating();
        inputs.gyroRate = gyro.getRate();
    }

    public void zeroRoll() {
        gyroZero = gyro.getRoll();
    }
}
