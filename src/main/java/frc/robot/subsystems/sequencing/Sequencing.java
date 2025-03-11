package frc.robot.subsystems.sequencing;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.fieldtracking.FieldTracking;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Sequencing extends SubsystemBase {
    private static final Distance ROBOT_X_LENGTH = Units.Inches.of(34.0);

    private final Elevator elevator;
    private final Intake intake;
    private final SwerveDrive swerveDrive;
    private final Manipulator manipulator;
    private final FieldTracking fieldTracking;
    private final Field2d field2d;
    private final SequencingIO io;
    private final SequencingIOInputsAutoLogged inputs = new SequencingIOInputsAutoLogged();
    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    @AutoLogOutput
    private ReefSides reefSide;

    @AutoLogOutput
    private LeftOrRight leftOrRight;

    @AutoLogOutput
    private ReefLevel reefLevel;

    // HashMap<Pair<ReefSides, LeftOrRight>, Pose2d> waypoints = new HashMap<>() {
    // {
    // put(Pair.of(ReefSides.Side1, LeftOrRight.Left), new Pose2d(0.0, 0.0,
    // Rotation2d.fromDegrees(0.0)));
    // put(Pair.of(ReefSides.Side1, LeftOrRight.Right), new Pose2d(0.0, 0.0,
    // Rotation2d.fromDegrees(0.0)));
    // put(Pair.of(ReefSides.Side2, LeftOrRight.Left), new Pose2d(3.985, 5.212,
    // Rotation2d.fromDegrees(-60.0)));
    // put(Pair.of(ReefSides.Side2, LeftOrRight.Right), new Pose2d(0.0, 0.0,
    // Rotation2d.fromDegrees(0.0)));
    // put(Pair.of(ReefSides.Side3, LeftOrRight.Left), new Pose2d(0.0, 0.0,
    // Rotation2d.fromDegrees(0.0)));
    // put(Pair.of(ReefSides.Side3, LeftOrRight.Right), new Pose2d(0.0, 0.0,
    // Rotation2d.fromDegrees(0.0)));
    // put(Pair.of(ReefSides.Side4, LeftOrRight.Left), new Pose2d(0.0, 0.0,
    // Rotation2d.fromDegrees(0.0)));
    // put(Pair.of(ReefSides.Side4, LeftOrRight.Right), new Pose2d(0.0, 0.0,
    // Rotation2d.fromDegrees(0.0)));
    // put(Pair.of(ReefSides.Side5, LeftOrRight.Left), new Pose2d(0.0, 0.0,
    // Rotation2d.fromDegrees(0.0)));
    // put(Pair.of(ReefSides.Side5, LeftOrRight.Right), new Pose2d(5.323, 2.953,
    // Rotation2d.fromDegrees(120.0)));
    // put(Pair.of(ReefSides.Side6, LeftOrRight.Left), new Pose2d(0.0, 0.0,
    // Rotation2d.fromDegrees(0.0)));
    // put(Pair.of(ReefSides.Side6, LeftOrRight.Right), new Pose2d(0.0, 0.0,
    // Rotation2d.fromDegrees(0.0)));
    // }
    // };

    /**
     * side 1 is closest to drivers station and is going clockwise
     */
    public enum ReefSides {
        Side1(18, 7),
        Side2(19, 6),
        Side3(20, 11),
        Side4(21, 10),
        Side5(22, 9),
        Side6(17, 8);

        public int blueAprilTagID;
        public int redAprilTagID;

        ReefSides(int blueAprilTagId, int redAprilTagId) {
            this.blueAprilTagID = blueAprilTagId;
            this.redAprilTagID = redAprilTagId;
        }
    }

    public enum LeftOrRight {
        Left,
        Right
    }

    public enum ReefLevel {
        L1,
        L2,
        L3,
        L4;

        Elevator.Setpoint setpoint() {
            return switch (this) {
                case L1 -> Elevator.Setpoint.L1;
                case L2 -> Elevator.Setpoint.L2;
                case L3 -> Elevator.Setpoint.L3;
                case L4 -> Elevator.Setpoint.L4;
            };
        }
    }

    public Sequencing(
            final Elevator elevator,
            final Intake intake,
            final SwerveDrive swerveDrive,
            final Manipulator manipulator,
            final FieldTracking fieldTracking,
            final SequencingIO io) {
        this.elevator = elevator;
        this.intake = intake;
        this.swerveDrive = swerveDrive;
        this.manipulator = manipulator;
        this.fieldTracking = fieldTracking;
        this.io = io;
        this.field2d = new Field2d();
        SmartDashboard.putData("Field", field2d);
        // SmartDashboard.putData("leftL1", selectTarget(LeftOrRight.Left,
        // ReefLevel.L1));
        // SmartDashboard.putData("leftL2", selectTarget(LeftOrRight.Left,
        // ReefLevel.L2));
        // SmartDashboard.putData("leftL3", selectTarget(LeftOrRight.Left,
        // ReefLevel.L3));
        // SmartDashboard.putData("leftL4", selectTarget(LeftOrRight.Left,
        // ReefLevel.L4));
        // SmartDashboard.putData("rightL1", selectTarget(LeftOrRight.Right,
        // ReefLevel.L1));
        // SmartDashboard.putData("rightL2", selectTarget(LeftOrRight.Right,
        // ReefLevel.L2));
        // SmartDashboard.putData("rightL3", selectTarget(LeftOrRight.Right,
        // ReefLevel.L3));
        // SmartDashboard.putData("rightL4", selectTarget(LeftOrRight.Right,
        // ReefLevel.L4));
    }

    @Override
    public void periodic() {
        field2d.setRobotPose(swerveDrive.getEstimatedPose());
        io.updateInputs(inputs);
        Logger.processInputs("Sequencing", inputs);

        listenSelect(inputs.leftL1, "leftL1", LeftOrRight.Left, ReefLevel.L1);
        listenSelect(inputs.leftL2, "leftL2", LeftOrRight.Left, ReefLevel.L2);
        listenSelect(inputs.leftL3, "leftL3", LeftOrRight.Left, ReefLevel.L3);
        listenSelect(inputs.leftL4, "leftL4", LeftOrRight.Left, ReefLevel.L4);
        listenSelect(inputs.rightL1, "rightL1", LeftOrRight.Right, ReefLevel.L1);
        listenSelect(inputs.rightL2, "rightL2", LeftOrRight.Right, ReefLevel.L2);
        listenSelect(inputs.rightL3, "rightL3", LeftOrRight.Right, ReefLevel.L3);
        listenSelect(inputs.rightL4, "rightL4", LeftOrRight.Right, ReefLevel.L4);
    }

    private void listenSelect(
            final boolean input, final String key, final LeftOrRight leftOrRight, final ReefLevel reefLevel) {
        if (input) {
            SmartDashboard.putBoolean(key, false);
            this.leftOrRight = leftOrRight;
            this.reefLevel = reefLevel;
        }

        // SmartDashboard.putBoolean(key, this.leftOrRight == leftOrRight &&
        // this.reefLevel == reefLevel);
    }

    public Command scoreCoral(final ReefSides side, final LeftOrRight leftOrRight, final ReefLevel level) {
        return runOnce(() -> {
            // Pose2d endPose = waypoints.get(Pair.of(side, leftOrRight));
            // if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
            // endPose = FlippingUtil.flipFieldPose(endPose);
            // }
            final Pose2d endPose = reefPose(side, leftOrRight);

            // Logger.recordOutput("Sequencing/EndPose", endPose);

            final List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(swerveDrive.getEstimatedPose(),
                    endPose);

            // if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)));
            // waypoints.set(1, waypoints.get(1).flip());

            final PathConstraints constraints = new PathConstraints(
                    0.5 * Constants.DRIVE_SPEED_MULTIPLIER, 0.2, 2 * Math.PI, 4 * Math.PI); // The constraints
            // for this path.

            final PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    List.of(),
                    List.of(),
                    List.of(),
                    // List.of(new ConstraintsZone(0.8, 1.0, new
                    // PathConstraints(0.5*Constants.DRIVE_SPEED_MULTIPLIER,
                    // 0.5, 2 * Math.PI, 4 * Math.PI))),
                    List.of(new EventMarker("Bob", .8, elevator.goToSetpoint(level.setpoint()))),
                    constraints,
                    null, // The ideal starting state, this is only relevant for pre-planned paths, so can
                    // be null for on-the-fly paths.
                    new GoalEndState(0.0, endPose.getRotation()), // Goal end state. You can set a holonomic rotation
                    // here. If using a differential drivetrain, the
                    // rotation will have no effect.
                    false);
            path.preventFlipping = true;

            final Pose2d[] points = path
                    .generateTrajectory(
                            swerveDrive.getChassisSpeeds(), swerveDrive.getGyroRotation(), swerveDrive.getConfig())
                    .getStates()
                    .stream()
                    .map(trajectory -> trajectory.pose)
                    .toArray(length -> new Pose2d[length]);
            Logger.recordOutput("Sequencing/Path", points);

            // System.out.println("!!!!!!!!!!!!!!!");

            // for (Waypoint waypoints2 : waypoints) {
            // System.out.println("WAYP " + waypoints2.anchor().toString());
            // }

            // for (PathPoint allPathPoints : path.getAllPathPoints()) {
            // System.out.println("POIT " + allPathPoints.position.toString());
            // }

            AutoBuilder.followPath(path)
                    .andThen((run(() -> {
                    })
                            .until(elevator::isAtSetpoint)
                            .andThen(manipulator.releaseCoral().withTimeout(.5))
                            .andThen(elevator.goToSetpoint(Elevator.Setpoint.Bottom)))
                            .deadlineFor(fieldTracking.maintainPose(endPose)))
                    .schedule();
        });
    }

    public Command followPath(final PathPlannerPath path) {
        return AutoBuilder.followPath(path)
                .beforeStarting(() -> field2d.getObject("path").setPoses(path.getPathPoses()))
                .andThen(() -> field2d.getObject("path").setPoses());
    }

    Command scoreCommand;

    public void bindScoreCoral(final Trigger t) {
        t.onTrue(runOnce(() -> {

            if (fieldTracking.isAprilTagDetected()) {
                long currentAprilTag = fieldTracking.getAprilTag();
                for (ReefSides reefSides : ReefSides.values()) {
                    if ((reefSides.blueAprilTagID == currentAprilTag && DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) 
                    || (reefSides.redAprilTagID == currentAprilTag && DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)) {
                        reefSide = reefSides;
                        cancelScoreCoral();
                        scoreCommand = scoreCoral(reefSide, leftOrRight, reefLevel);
                    }
                }

            }
        }));
    }

    public void cancelScoreCoral(){
        if (scoreCommand != null && !scoreCommand.isFinished()) {
            scoreCommand.cancel();
        }
    }

    public Command selectTarget(final LeftOrRight leftOrRight, final ReefLevel reefLevel) {
        return runOnce(() -> {
            this.leftOrRight = leftOrRight;
            this.reefLevel = reefLevel;

            // onChanged() {
            if (scoreCommand != null && !scoreCommand.isFinished()) {
                scoreCommand.cancel();
                scoreCommand = scoreCoral(reefSide, leftOrRight, reefLevel);
            }
            // }
        });
    }

    public Pose2d reefPose(final ReefSides reefSides, final LeftOrRight leftOrRight) {
        int aprilTagID;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            aprilTagID = reefSides.blueAprilTagID;
        } else {
            aprilTagID = reefSides.redAprilTagID;
        }

        // the april tag ids hard coded into the enum are assumed to be valid
        assert fieldLayout.getTagPose(aprilTagID).isPresent();

        final Pose2d aprilTagPose = fieldLayout.getTagPose(aprilTagID).get().toPose2d(); // 3d no no

        return aprilTagPose.plus(new Transform2d(
                ROBOT_X_LENGTH.div(2),
                Units.Inches.of(leftOrRight == LeftOrRight.Left ? -6.5 : 6.5),
                Rotation2d.k180deg));
    }
}
