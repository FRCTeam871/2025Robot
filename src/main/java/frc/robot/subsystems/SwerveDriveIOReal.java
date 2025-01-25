package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveDriveIOReal implements SwerveDriveIO {
    AHRS gyro;
    public SwerveDriveIOReal( AHRS gyro){
        this.gyro = gyro;
    }
    @Override
    public void updateInputs(SwerveDriveIOInputs inputs){
        inputs.gyroRotation = gyro.getRotation2d();
        inputs.isCalibrating = gyro.isCalibrating();
        inputs.gyroRate = gyro.getRate();
    }
}
