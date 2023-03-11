// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.revrobotics.jni.*;



import frc.robot.Subsystems.DriveBase;
import frc.robot.Controls;
public class Robot extends TimedRobot {

private Command autoSelected;
private RobotContainer robotContainer;

private boolean precision = false;
private double throttle = 0;
private double reverse = 0;
private boolean brake = false;
private double turn = 0;
private boolean rotate  = false;
private double sensitivity = 0.5;
private double left = 0;
private double right  = 0;

Robot() {
  super(0.05);
}

/**
 * This function is run when the robot is first started up and should be used
 * for any initialization code.
 */
@Override
public void robotInit() {
  
  robotContainer = new RobotContainer();
}

/**
 * This function is called every robot packet, no matter the mode. Use this for
 * items like diagnostics that you want ran during disabled, autonomous,
 * teleoperated and test.
 *
 * <p>
 * This runs after the mode specific periodic functions, but before LiveWindow
 * and SmartDashboard integrated updating.
 */
@Override
public void robotPeriodic() {
  CommandScheduler.getInstance().run();
}

@Override
public void autonomousInit() {
  autoSelected = robotContainer.getAutonomousCommand();

  if(autoSelected != null) {
    autoSelected.schedule();
  }
}

/**
 * This function is called periodically during autonomous.
 */
@Override
public void autonomousPeriodic() {}

@Override
public void teleopInit() {
  if(autoSelected != null) {
    autoSelected.cancel();
  }
}

/**
 * This function is called periodically during operator control.
 */
@Override
public void teleopPeriodic() {

  if(!robotContainer.updateDrivetrain())
    robotContainer.getDriveBase().drive(0, 0);
  else {
    /** 
    throttle = Controls.driver.getRightTriggerAxis();
    reverse = Controls.driver.getLeftTriggerAxis();
    turn = Controls.driver.getLeftX();
    precision = Controls.driver.getRightBumper();
    rotate = Controls.driver.getLeftBumper();
      //Braking
       /*** Precision ***/
      //Hold for Precision Speed
    if(precision){
      sensitivity = Constants.ControlConstants.DRIVE_SENSITIVITY_PRECISION;
    }
      //Release for Regular Speed
    else{
      sensitivity = Constants.ControlConstants.DRIVE_SENSITIVITY_DEFAULT;
    }

    /*** Driving ***/
      //Braking
   
        //Pirouetting (Turn in place). 
      if(rotate){
          //If the joystick is pushed passed the threshold. 
        if(Math.abs(turn) > Constants.ControlConstants.AXIS_THRESHOLD){
            //Sets it to spin the desired direction.
          left =  turn;
          right =  (turn * -1);
        }
          //If its not past the threshold stop spinning
        else if(Math.abs(turn) < Constants.ControlConstants.AXIS_THRESHOLD){
          left = 0;
          right = 0;
        }
      }
        //Not pirouetting (Not turning in place).
      else{
          //Turning right
        if(turn > Constants.ControlConstants.AXIS_THRESHOLD){
            //Makes left slow down by a factor of how far the axis is pushed. 
          left = (throttle - reverse) * sensitivity;
          right = (throttle - reverse) * sensitivity * (1 - turn);
        }
          //Turning left
        else if(turn < (-1 * Constants.ControlConstants.AXIS_THRESHOLD)){
            //Makes right speed up by a factor of how far the axis is pushed. 
          left = (throttle - reverse) * sensitivity  * (1 + turn);
          right = (throttle - reverse) * sensitivity;
        }
          //Driving straight 
        else{
            //No joystick manipulation. 
          left = (throttle - reverse) * sensitivity;
          right = (throttle - reverse) * sensitivity;
        }
      }
      
    
      //After speed manipulation, send to drivebase. 
    robotContainer.getDriveBase().drive(Controls.driver.getLeftTriggerAxis(), Controls.driver.getRightTriggerAxis());
  }
if(Controls.driver.getRightBumper() && Controls.driver.getAButton() ){
  robotContainer.getDriveBase().rotateArm(0.25*-1, Controls.driver.getAButton());
}
else if(Controls.driver.getAButton()){
  robotContainer.getDriveBase().rotateArm(0.25, Controls.driver.getAButton());

}
else{
  robotContainer.getDriveBase().rotateArm(0, Controls.driver.getAButton());
}
if(Controls.driver.getBackButton() && Controls.driver.getBButton() ){
  robotContainer.getDriveBase().translateArm(0.4*-1, Controls.driver.getBButton());
}
else if(Controls.driver.getBButton()){
  robotContainer.getDriveBase().translateArm(0.4, Controls.driver.getBButton());

}
else{
  robotContainer.getDriveBase().translateArm(0, Controls.driver.getBButton());
}

if (Controls.driver.getStartButton()){
  robotContainer.getDriveBase().resetArmEncoderDistance();
}
//robotContainer.getDriveBase().printEncoderDistances();
if(Controls.driver.getYButton()){
  robotContainer.getDriveBase().pidRotateArm();
  robotContainer.getDriveBase().pidTranslateArm();

}
}

@Override
public void disabledPeriodic() {}

@Override
public void disabledInit() {}

@Override
public void testInit() {}


@Override
public void testPeriodic() {}
}
