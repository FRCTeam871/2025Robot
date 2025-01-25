package frc.robot.subsystems;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.ElevatorIO;

public class ElevatorIOReal implements ElevatorIO{

    private final double ZERO_VOLTAGE_VALUE = 0; //TODO: figure out this number
    private final double INCHES_PER_VOLT = 0; //TODO: Figure out magic numer
    private final SparkAnalogSensor encoder;
    private final LinearFilter encoderFilter;

    SparkFlex elevatorMotor;

    public ElevatorIOReal() {
        elevatorMotor = new SparkFlex(1000, MotorType.kBrushless);
        encoder = elevatorMotor.getAnalog();

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
