package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

public interface ManipulatorIO {
    public default void pushCoral(boolean extend){
      Logger.recordOutput("Manipulator/PushPiston", extend);  
    }
    public default void holdCoral(boolean extend){
      Logger.recordOutput("Manipulator/HoldPiston", extend);
    }
}
