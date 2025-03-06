package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {
   
    private final double INPUT_BOTTOM = 4.43;
    private final double INPUT_TOP = .072;
    private final Distance OUTPUT_BOTTOM = Units.Inches.of(18.25);
    private final Distance OUTPUT_TOP = Units.Inches.of(77.25);
    private final double SLOPE =
            (OUTPUT_TOP.in(Units.Inches) - OUTPUT_BOTTOM.in(Units.Inches)) / (INPUT_TOP - INPUT_BOTTOM);
    private final double INTERCEPT = OUTPUT_TOP.in(Units.Inches) - (SLOPE * INPUT_TOP);
     //TODO: find relative input actual
    private final double RELATIVE_INPUT_BOTTOM = -27.46;
    private final double RELATIVE_INPUT_TOP = 176.72;
    private final double RELATIVE_SLOPE = 
    (OUTPUT_TOP.in(Units.Inches) - OUTPUT_BOTTOM.in(Units.Inches)) / (RELATIVE_INPUT_TOP - RELATIVE_INPUT_BOTTOM);
    private final double RELATIVE_INTERCEPT = OUTPUT_TOP.in(Units.Inches) - (SLOPE * RELATIVE_INPUT_TOP);

    private final SparkAnalogSensor absolutePot;
    private final LinearFilter absolutePotFilter;

    private final LinearFilter relativeFilter;
    private final RelativeEncoder relativeEncoder;
    private final double RELATIVE_ENCODER_CONVERSION = 1.0;

    private Distance relativeEncoderZero = Units.Inches.of(0);

    final SparkFlex elevatorMotor;

    public ElevatorIOReal() {
        this.elevatorMotor = new SparkFlex(13, MotorType.kBrushless);
        
        this.absolutePot = elevatorMotor.getAnalog();
        this.absolutePotFilter = LinearFilter.singlePoleIIR(.2, .02);
        this.relativeEncoder = elevatorMotor.getEncoder();
        this.relativeFilter = LinearFilter.singlePoleIIR(.2, .02);

        final SparkFlexConfig config = new SparkFlexConfig();
        // config.smartCurrentLimit(40);
        config.idleMode(IdleMode.kBrake);
        // TODO: current limit
        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        Logger.recordOutput("Elevator/Current", elevatorMotor.getOutputCurrent());
        final double rawRelativeEncoder = (relativeEncoder.getPosition() *RELATIVE_SLOPE) + RELATIVE_INTERCEPT;
        inputs.currentHeightRelative = Units.Inches.of(relativeFilter.calculate(rawRelativeEncoder)).minus(relativeEncoderZero);
        Logger.recordOutput("Elevator/RelativeEncoder", relativeEncoder.getPosition());
        final double rawEncoder = (absolutePot.getPosition() * SLOPE) + INTERCEPT;
        Logger.recordOutput("Elevator/RawAbsolutePot", absolutePot.getPosition());
        inputs.currentHeight = Units.Inches.of(absolutePotFilter.calculate(rawEncoder));
        Logger.recordOutput("Elevator/CurrentHeightInches", inputs.currentHeight.in(Inches));
        inputs.currentHeightNormalized = inputs.currentHeight.minus(OUTPUT_BOTTOM).div(OUTPUT_TOP.minus(OUTPUT_BOTTOM)).in(Units.Value);
    }

    @Override
    public void setElevatorSpeed(double speed) {
        elevatorMotor.set(speed);
        ElevatorIO.super.setElevatorSpeed(speed);
    }

    public void resetRelativeEncoder(ElevatorIOInputs inputs){
     relativeEncoderZero = (inputs.currentHeightRelative.plus(relativeEncoderZero)).minus(inputs.currentHeight);
     Logger.recordOutput("Elevator/RelativeEncoderZero", relativeEncoderZero);
    }
}
