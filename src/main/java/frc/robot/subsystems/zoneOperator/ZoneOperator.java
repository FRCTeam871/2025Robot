package frc.robot.subsystems.zoneOperator;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Condition;
import java.util.function.Function;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.fieldtracking.FieldTracking;
import frc.robot.subsystems.sequencing.Sequencing;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class ZoneOperator extends SubsystemBase {

    class Zone {
        // copeputer numerical copetrol
        Command command;
        Function<Pose2d, Boolean> condition;
        String label;

        public Zone(Function<Pose2d, Boolean> condition, Command command, String label) {
            this.condition = condition;
            this.command = command;
            this.label = label;
        }

        private void zonePeriodic() {
            if (command == null || condition == null) {
                return;
            }
            Pose2d pose;
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                pose = FlippingUtil.flipFieldPose(swerveDrive.getEstimatedPose());
            } else {
                pose = swerveDrive.getEstimatedPose();
            }

            if (condition.apply(pose) && !command.isScheduled()) {
                command.schedule();
                System.out.println("zone canceled -> " + label);
            } else if (!condition.apply(pose) && command.isScheduled()) {
                command.cancel();
                System.out.println("zone canceled -> " + label);
            }
        }
    }

    List<Zone> zoneList;
    SwerveDrive swerveDrive;

    // Elevator elevator;
    // Sequencing sequencing;
    // FieldTracking fieldTracking;
    public ZoneOperator(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        zoneList = new ArrayList<Zone>();
    }

    /**From blue teams perspective */
    public void addZone(Function<Pose2d, Boolean> predicate, Command command, String label) {
        Zone zone = new Zone(predicate, command, label);
        zoneList.add(zone);
    }

    /**From blue teams perspective */
    public void addCircle(Translation2d center, Distance radius, Command command, String label) {
        this.addZone((currentPose) -> {
            if (center.getDistance(currentPose.getTranslation()) < radius.in(Meters)) {
                return true;
            } else {
                return false;
            }

        }, command, label);

    }

    @Override
    public void periodic() {
        for (Zone zone : zoneList) {
            zone.zonePeriodic();
        }

    }

}
