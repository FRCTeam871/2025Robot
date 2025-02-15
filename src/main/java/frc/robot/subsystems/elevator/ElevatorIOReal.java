package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {

    private final double INPUT_BOTTOM = 4.4315;
    private final double INPUT_TOP = .633;
    private final Distance OUTPUT_BOTTOM = Units.Inches.of(18.25);
    private final Distance OUTPUT_TOP = Units.Inches.of(77.25);
    private final double SLOPE =
            (OUTPUT_TOP.in(Units.Inches) - OUTPUT_BOTTOM.in(Units.Inches)) / (INPUT_TOP - INPUT_BOTTOM);
    private final double INTERCEPT = OUTPUT_TOP.in(Units.Inches) - (SLOPE * INPUT_TOP);

    private final SparkAnalogSensor encoder;
    private final LinearFilter encoderFilter;

    SparkFlex elevatorMotor;

    public ElevatorIOReal() {
        elevatorMotor = new SparkFlex(13, MotorType.kBrushless);
        encoder = elevatorMotor.getAnalog();
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        // TODO: current limit
        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoderFilter = LinearFilter.singlePoleIIR(.2, .02);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        final double rawEncoder = (encoder.getPosition() * SLOPE) + INTERCEPT;
        Logger.recordOutput("Elevator/RawEncoder", encoder.getPosition());
        inputs.currentHeight = Units.Inches.of(encoderFilter.calculate(rawEncoder));
        Logger.recordOutput("Elevator/CurrentHeightInches", inputs.currentHeight.in(Inches));
    }

    @Override
    public void setElevatorSpeed(double speed) {
        elevatorMotor.set(speed);
        ElevatorIO.super.setElevatorSpeed(speed);
    }
}
