package frc.robot.Utilities.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class JoystickAxis extends Axis {
  private final GenericHID m_joystick;
  private final int m_axisnumber;

  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public JoystickAxis(GenericHID joystick, int axisNumber) {

    m_joystick = joystick;
    m_axisnumber = axisNumber;
  }

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button
   */
  public double getAxis() {
    return m_joystick.getRawAxis(m_axisnumber);
  }

  @Override
  public boolean getAsBoolean(){
    return (getAxis()<-Constants.ControlConstants.AXIS_THRESHOLD||getAxis()>Constants.ControlConstants.AXIS_THRESHOLD);
  }
}
