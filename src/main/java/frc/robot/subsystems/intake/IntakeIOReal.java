package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class IntakeIOReal implements IntakeIO {
    private final LinearFilter topXFilter = LinearFilter.singlePoleIIR(.06, .02);
    private final LinearFilter topYFilter = LinearFilter.singlePoleIIR(.06, .02);
    private final LinearFilter bottomXFilter = LinearFilter.singlePoleIIR(.06, .02);
    private final LinearFilter bottomYFilter = LinearFilter.singlePoleIIR(.06, .02);
    private final LinearFilter rightXFilter = LinearFilter.singlePoleIIR(.06, .02);
    private final LinearFilter rightYFilter = LinearFilter.singlePoleIIR(.06, .02);
    private final LinearFilter leftXFilter = LinearFilter.singlePoleIIR(.06, .02);
    private final LinearFilter leftYFilter = LinearFilter.singlePoleIIR(.06, .02);

    private final LoggedMechanism2d mechanism2d;
    private final LoggedMechanismLigament2d mechanismLeft;
    private final LoggedMechanismLigament2d mechanismRight;
    private final LoggedMechanismLigament2d mechanismPistonLeft;
    private final LoggedMechanismLigament2d mechanismPistonRight;

    private final DoubleSolenoid pistonLeft;
    private final DoubleSolenoid pistonRight;

    public IntakeIOReal() {
        this.pistonLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        this.pistonRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
        this.mechanism2d = new LoggedMechanism2d(18.5, 14.5);
        this.mechanismLeft = mechanism2d
                .getRoot("left", 6, 0)
                .append(new LoggedMechanismLigament2d(
                        "left ligament", 15.74, 180 - 66.7, 50, new Color8Bit(Color.kYellow)));
        this.mechanismRight = mechanism2d
                .getRoot("right", 18.5 - 6, 0)
                .append(new LoggedMechanismLigament2d("right ligament", 15.74, 66.7, 50, new Color8Bit(Color.kGray)));
        this.mechanismPistonLeft = mechanism2d
                .getRoot("left piston start", 3, 0)
                .append(new LoggedMechanismLigament2d("left piston", 6, 90, 30, new Color8Bit(Color.kAliceBlue)));
        this.mechanismPistonRight = mechanism2d
                .getRoot("Right piston start", 18.5 - 3, 0)
                .append(new LoggedMechanismLigament2d("Right piston", 6, 90, 30, new Color8Bit(Color.kBurlywood)));
    }

    @Override
    public void updateInputs(final IntakeIOInputs inputs) {
        NetworkTable limelightIntakeTable = NetworkTableInstance.getDefault().getTable("limelight-indexer");

        if (!limelightIntakeTable.containsKey("tcornxy")) {
            return; // ABORT MISSION
        }

        final double[] corn = limelightIntakeTable.getEntry("tcornxy").getDoubleArray(new double[8]);

        if (corn.length >= 8) {
            final Translation2d bottomRight = new Translation2d(corn[0], corn[1]);
            final Translation2d bottomLeft = new Translation2d(corn[2], corn[3]);
            final Translation2d topLeft = new Translation2d(corn[4], corn[5]);
            final Translation2d topRight = new Translation2d(corn[6], corn[7]);

            Translation2d top = topLeft.interpolate(topRight, 0.5);
            top = new Translation2d(topXFilter.calculate(top.getX()), topYFilter.calculate(top.getY()));

            Translation2d right = topRight.interpolate(bottomRight, 0.5);
            right = new Translation2d(rightXFilter.calculate(right.getX()), rightYFilter.calculate(right.getY()));

            Translation2d bottom = bottomLeft.interpolate(bottomRight, 0.5);
            bottom = new Translation2d(bottomXFilter.calculate(bottom.getX()), bottomYFilter.calculate(bottom.getY()));

            Translation2d left = bottomLeft.interpolate(topLeft, 0.5);
            left = new Translation2d(leftXFilter.calculate(left.getX()), leftYFilter.calculate(left.getY()));

            Logger.recordOutput("Intake/Midpoints", top, left, right, bottom);

            double height = top.getDistance(bottom);
            double width = left.getDistance(right);

            if (height > width) {
                inputs.tiltedRight = top.getX() < bottom.getX();
            } else {
                inputs.tiltedRight = right.getY() > left.getY();
            }
        } else {
            // purposefully leave inputs.tiltedRight the same as last update
        }

        inputs.isTargetValid = limelightIntakeTable.getEntry("tv").getInteger(0) == 1;
        inputs.timeStamp = NetworkTablesJNI.now();

        Logger.recordOutput("Intake/Mechanism", mechanism2d);
    }

    @Override
    public void setLeftPistonOut(final boolean extend) {
        IntakeIO.super.setLeftPistonOut(extend); // ??
        mechanismPistonLeft.setLength(extend ? 9 : 6);
        if (extend) {
            pistonLeft.set(DoubleSolenoid.Value.kReverse);
        } else {
            pistonLeft.set(DoubleSolenoid.Value.kForward);
        }
    }

    @Override
    public void setRightPistonOut(final boolean extend) {
        IntakeIO.super.setRightPistonOut(extend);
        mechanismPistonRight.setLength(extend ? 9 : 6);
        if (extend) {
            pistonRight.set(DoubleSolenoid.Value.kReverse);
        } else {
            pistonRight.set(DoubleSolenoid.Value.kForward);
        }
    }
}
