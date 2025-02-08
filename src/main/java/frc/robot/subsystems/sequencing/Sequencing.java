package frc.robot.subsystems.sequencing;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.fieldtracking.FieldTracking;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class Sequencing extends SubsystemBase {
    private static final Distance ROBOT_X_LENGTH = Units.Inches.of(34.0);

    Elevator elevator;
    Intake intake;
    SwerveDrive swerveDrive;
    Manipulator manipulator;
    FieldTracking fieldTracking;
    Field2d field2d;
    SequencingIO io;
    SequencingIOInputsAutoLogged inputs = new SequencingIOInputsAutoLogged();
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    @AutoLogOutput
    ReefSides reefSide;

    @AutoLogOutput
    LeftOrRight leftOrRight;

    @AutoLogOutput
    ReefLevel reefLevel;

    // HashMap<Pair<ReefSides, LeftOrRight>, Pose2d> waypoints = new HashMap<>() {
    //     {
    //         put(Pair.of(ReefSides.Side1, LeftOrRight.Left), new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    //         put(Pair.of(ReefSides.Side1, LeftOrRight.Right), new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    //         put(Pair.of(ReefSides.Side2, LeftOrRight.Left), new Pose2d(3.985, 5.212, Rotation2d.fromDegrees(-60.0)));
    //         put(Pair.of(ReefSides.Side2, LeftOrRight.Right), new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    //         put(Pair.of(ReefSides.Side3, LeftOrRight.Left), new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    //         put(Pair.of(ReefSides.Side3, LeftOrRight.Right), new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    //         put(Pair.of(ReefSides.Side4, LeftOrRight.Left), new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    //         put(Pair.of(ReefSides.Side4, LeftOrRight.Right), new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    //         put(Pair.of(ReefSides.Side5, LeftOrRight.Left), new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    //         put(Pair.of(ReefSides.Side5, LeftOrRight.Right), new Pose2d(5.323, 2.953, Rotation2d.fromDegrees(120.0)));
    //         put(Pair.of(ReefSides.Side6, LeftOrRight.Left), new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    //         put(Pair.of(ReefSides.Side6, LeftOrRight.Right), new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    //     }
    // };

    public Sequencing(Elevator elevator, Intake intake, SwerveDrive swerveDrive, Manipulator manipulator,
            FieldTracking fieldTracking, SequencingIO io) {
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

    /**
     * side 1 is closest to drivers station and is going clockwise
     */
    public enum ReefSides {
        Side1(18, 7), 
        Side2(19, 6), 
        Side3(20,11), 
        Side4(21, 10), 
        Side5(22, 9), 
        Side6(17,8);

        public int blueAprilTagID;
        public int redAprilTagID;

        ReefSides(int blueAprilTagId, int redAprilTagId) {
            this.blueAprilTagID = blueAprilTagId;
            this.redAprilTagID = redAprilTagId;
        }
    }

    public enum LeftOrRight {
        Left, Right
    }

    public enum ReefLevel {
        L1, L2, L3, L4;

        Elevator.Setpoint setpoint() {
            return switch (this) {
                case L1 -> Elevator.Setpoint.L1;
                case L2 -> Elevator.Setpoint.L2;
                case L3 -> Elevator.Setpoint.L3;
                case L4 -> Elevator.Setpoint.L4;
            };
        }
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

    private void listenSelect(boolean input, String key, LeftOrRight leftOrRight, ReefLevel reefLevel) {
        if (input) {
            SmartDashboard.putBoolean(key, false);
            this.leftOrRight = leftOrRight;
            this.reefLevel = reefLevel;
        }

        // SmartDashboard.putBoolean(key, this.leftOrRight == leftOrRight &&
        // this.reefLevel == reefLevel);
    }

    public Command scoreCoral(ReefSides side, LeftOrRight leftOrRight, ReefLevel level) {
        return runOnce(() -> {
            // Pose2d endPose = waypoints.get(Pair.of(side, leftOrRight));
            // if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
            //     endPose = FlippingUtil.flipFieldPose(endPose);
            // }
            Pose2d endPose = reefPose(side, leftOrRight);



            // Logger.recordOutput("Sequencing/EndPose", endPose);

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(swerveDrive.getEstimatedPose(), endPose);

            // if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)));
            // waypoints.set(1, waypoints.get(1).flip());

            PathConstraints constraints = new PathConstraints(0.5*Constants.DRIVE_SPEED_MULTIPLIER, 0.2, 2 * Math.PI, 4 * Math.PI); // The constraints
                                                                                                   // for this path.

            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    List.of(),
                    List.of(),
                    List.of(),
                    // List.of(new ConstraintsZone(0.8, 1.0, new PathConstraints(0.5*Constants.DRIVE_SPEED_MULTIPLIER, 0.5, 2 * Math.PI, 4 * Math.PI))),
                    List.of(new EventMarker("Bob", .8, elevator.goToSetpoint(level.setpoint()))),
                    constraints,
                    null, // The ideal starting state, this is only relevant for pre-planned paths, so can
                          // be null for on-the-fly paths.
                    new GoalEndState(0.0, endPose.getRotation()), // Goal end state. You can set a holonomic rotation
                                                                  // here. If using a differential drivetrain, the
                                                                  // rotation will have no effect.
                    false);
            path.preventFlipping = true;

            Pose2d[] points = path.generateTrajectory(swerveDrive.getChassisSpeeds(), swerveDrive.getRotation(), swerveDrive.getConfig()).getStates().stream().map(trajectory-> trajectory.pose).toArray(length -> new Pose2d[length] );
            Logger.recordOutput("Sequencing/Path", points);

            // System.out.println("!!!!!!!!!!!!!!!");

            // for (Waypoint waypoints2 : waypoints) {
            //     System.out.println("WAYP " + waypoints2.anchor().toString());
            // }


            // for (PathPoint allPathPoints : path.getAllPathPoints()) {
            //     System.out.println("POIT " + allPathPoints.position.toString());
            // }

            AutoBuilder.followPath(path)
                    .andThen(
                            (run(() -> {
                            })
                                    .until(elevator::isAtSetpoint)
                                    .andThen(manipulator.sendHoldPistonIn())
                                    .andThen(elevator.goToSetpoint(Elevator.Setpoint.Bottom)))
                                    .deadlineFor(fieldTracking.maintainPose(endPose)))
                    .schedule();
        });
    }

    public Command followPath(PathPlannerPath path) {
        return AutoBuilder.followPath(path)
                .beforeStarting(() -> field2d.getObject("path").setPoses(path.getPathPoses()))
                .andThen(() -> field2d.getObject("path").setPoses());

    }

    Command scoreCommand;

    public void bindScoreCoral(Trigger t) {
        t.onTrue(runOnce(() -> {
            // TODO: set reefSide based on apriltag

            if (scoreCommand != null && !scoreCommand.isFinished())
                scoreCommand.cancel();
            scoreCommand = scoreCoral(reefSide, leftOrRight, reefLevel);

        }));

    }

    public Command selectTarget(LeftOrRight leftOrRight, ReefLevel reefLevel) {
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

    public Pose2d reefPose(ReefSides reefSides, LeftOrRight leftOrRight) {
        int aprilTagID;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            aprilTagID = reefSides.blueAprilTagID;
        } else{
            aprilTagID = reefSides.redAprilTagID;
        }
        
        Pose2d aprilTagPose = fieldLayout.getTagPose(aprilTagID).get().toPose2d(); //3d no no

        return aprilTagPose.plus(new Transform2d(
            ROBOT_X_LENGTH.div(2), 
            Units.Inches.of(leftOrRight == LeftOrRight.Left ? -6.5 : 6.5), 
            Rotation2d.k180deg)
        );
    }
}
