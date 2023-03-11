package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.DriveBase;

public class DoNothing extends SequentialCommandGroup {
  
  boolean doneCommand;
  DriveBase drivebase;
  /**
   * Add your docs here.
   */
  public DoNothing(DriveBase passedDrivebase) {
    drivebase = passedDrivebase;

  }


  @Override
  public boolean runsWhenDisabled() {
      return false;
  }
}