package frc;

import java.text.FieldPosition;
import java.util.ArrayList;
import java.util.SortedMap;
import java.util.function.Function;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.config.SmartMotionConfigAccessor;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.sequencing.SequencingIO;
import frc.robot.subsystems.sequencing.Sequencing;
import frc.robot.subsystems.sequencing.Sequencing.LeftOrRight;
import frc.robot.subsystems.sequencing.Sequencing.ReefLevel;
import frc.robot.subsystems.sequencing.Sequencing.ReefSides;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class AutonomousPlanner {

    public enum FieldPosition {
        StartLeft, StartMiddle, StartRight,

        ReefSide1, ReefSide2, ReefSide3,
        ReefSide4, ReefSide5, ReefSide6,

        CoralStationLeft, CoralStationRight
    }

    private final Sequencing sequencing;
    private final Manipulator manipulator;
    private final SwerveDrive swerveDrive;

    private final ArrayList<SendableChooser<FieldPosition>> positions = new ArrayList<>();
    private final ArrayList<SendableChooser<Function<ReefSides, Command>>> actions = new ArrayList<>();
    private final SendableChooser<FieldPosition> start;

    public AutonomousPlanner(Elevator elevator, Intake intake, Manipulator manipulator, SwerveDrive swerveDrive, Sequencing sequencing) {
        this.sequencing = sequencing;
        this.manipulator = manipulator;
        this.swerveDrive = swerveDrive;

        this.start = positionDropdown();
        // how make this into elastic???

        // goes for how many auton stages there are
        // TODO find a better number for this
        for (int i = 0; i < 7; i++) {
            final SendableChooser<FieldPosition> position = positionDropdown();
            positions.add(position);
            SmartDashboard.putData("Auton/Position" + i, position);
            final SendableChooser<Function<ReefSides, Command>> action = actionDropdown();
            actions.add(action);
            SmartDashboard.putData("Auton/Action", action);
        }
    }

    private SendableChooser<FieldPosition> positionDropdown() {
        SendableChooser<FieldPosition> chooser = new SendableChooser<>();

        for (FieldPosition pos : FieldPosition.values()) {
            chooser.addOption(pos.toString(), pos);
        }
        return chooser;
    }
        
    private SendableChooser<Function<ReefSides, Command>> actionDropdown() {
        SendableChooser<Function<ReefSides, Command>> choose = new SendableChooser<>();
        choose.addOption("None", side -> Commands.none());
        for(ReefLevel level : ReefLevel.values()){
            choose.addOption(level + " Left" ,side -> sequencing.scoreCoral(side, LeftOrRight.Left, level));
        }
        return choose;
    }

    public PathPlannerPath loadPath(String pathFileName) {
        try {
            return PathPlannerPath.fromPathFile(pathFileName);
        } catch (Exception e) {
            return null;
        }
    }

    public Command generateCommand() {
        final SequentialCommandGroup scg = new SequentialCommandGroup();
        FieldPosition currentPosition = start.getSelected();
        for (int i = 0; i < 7; i++) {
            boolean continueLoop = generateStage(scg, currentPosition, positions.get(i).getSelected());
            if (!continueLoop) {
                scg.addCommands(swerveDrive.manualDrive(() -> 0, () -> 0, () -> 0));
                break;
            }
        }
        return scg;
    }

    private boolean generateStage(final SequentialCommandGroup scg, final FieldPosition start,
            final FieldPosition end) {
        String pathFileName = start + "-" + end;
        PathPlannerPath path = loadPath(pathFileName.toLowerCase());
        System.out.println(pathFileName + " " + path);

        if (path == null) {
            return false;
        }
        return true;
    }
}