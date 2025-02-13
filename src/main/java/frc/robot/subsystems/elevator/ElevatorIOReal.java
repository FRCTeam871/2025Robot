package frc.robot.subsystems.elevator;

import java.lang.module.Configuration;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;

public class ElevatorIOReal implements ElevatorIO{

    private final double ZERO_VOLTAGE_VALUE = .112; 
    private final double INCHES_PER_VOLT = 1 / -.0931; 
    private final SparkAnalogSensor encoder;
    private final LinearFilter encoderFilter;

    SparkFlex elevatorMotor;

    public ElevatorIOReal() {
        //TODO: brake mode
        elevatorMotor = new SparkFlex(13, MotorType.kBrushless);
        encoder = elevatorMotor.getAnalog();    
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoderFilter = LinearFilter.singlePoleIIR(.08, .02);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        final double rawEncoder = (encoder.getPosition() - ZERO_VOLTAGE_VALUE) * INCHES_PER_VOLT;
        inputs.currentHeight = Units.Inches.of(encoderFilter.calculate(rawEncoder));
    }


    @Override
    public void setElevatorSpeed(double speed) {
        elevatorMotor.set(speed);
        ElevatorIO.super.setElevatorSpeed(speed);
    }
}