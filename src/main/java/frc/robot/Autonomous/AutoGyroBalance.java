package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.DriveBase;
import java.lang.*;

public class AutoGyroBalance extends SequentialCommandGroup {
  
  boolean doneCommand;
  DriveBase drivebase;
  double speed;

  public AutoGyroBalance(DriveBase passedDrivebase, double passedSpeed) {
    drivebase = passedDrivebase;
    speed = passedSpeed;
    addCommands(
    new AutoGyroBalance(drivebase, speed)
   );

  } 
}