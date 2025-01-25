package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    ManipulatorIO io;
    FieldTracking fieldTracking;
    public Manipulator(ManipulatorIO io, FieldTracking fieldTracking){
        this.io = io;
        this.fieldTracking = fieldTracking;
    }

   
    public Command sendPushPistonIn() {
        return run(() -> io.pushCoral(true)).finallyDo(canceled -> io.pushCoral(false)); 
    }
    public Command sendHoldPistonIn(){
        return run(() -> io.holdCoral(true)).finallyDo(canceled -> io.holdCoral(false));
    }

    public Command scoreCoral(){
        if(fieldTracking.followAprilTag().isFinished()){
        return run(()-> sendHoldPistonIn().andThen(sendPushPistonIn()));
        } else{
        return run(() -> {});
        }

    }
}
