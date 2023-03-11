package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Utilities.controllers.JoystickAxis;

public class Controls {
  // The driver's controller
  public static XboxController driver = new XboxController(Constants.ControlConstants.XBOX_CONTROLLER_DRIVER);
  public static XboxController operator = new XboxController(Constants.ControlConstants.XBOX_CONTROLLER_OPERATOR);

   //Buttons (Driver)
   static JoystickButton xboxA_Driver = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_A);
   static JoystickButton xboxB_Driver			  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_B);
   static JoystickButton xboxX_Driver			  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_X);
   static JoystickButton xboxY_Driver			  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_Y);
   static JoystickButton xboxLeftBumper_Driver  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_LEFT_BUMPER);
   static JoystickButton xboxRightBumper_Driver = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_RIGHT_BUMPER);
   static JoystickButton xboxBack_Driver		  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_BACK);
   static JoystickButton xboxStart_Driver		  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_START);
   static JoystickButton xboxL3_Driver		  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_L3);
   static JoystickButton xboxR3_Driver		  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_R3);
   
   //Buttons (Operator)
   static JoystickButton xboxA_Operator			= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_A);
   static JoystickButton xboxB_Operator			= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_B);
   static JoystickButton xboxX_Operator			= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_X);
   static JoystickButton xboxY_Operator			= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_Y);
   static JoystickButton xboxLeftBumper_Operator  = new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_LEFT_BUMPER);
   static JoystickButton xboxRightBumper_Operator = new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_RIGHT_BUMPER);
   static JoystickButton xboxBack_Operator		= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_BACK);
   static JoystickButton xboxStart_Operator		= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_START);
   static JoystickButton xboxL3_Operator		  	= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_L3);
   static JoystickButton xboxR3_Operator		  	= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_R3);

   //Axis (Driver)
  static JoystickAxis xboxRT_Driver = new JoystickAxis(driver, Constants.ControlConstants.XBOX_AXIS_RIGHT_TRIGGER);
  static JoystickAxis xboxLT_Driver = new JoystickAxis(driver, Constants.ControlConstants.XBOX_AXIS_LEFT_TRIGGER);
  static JoystickAxis xboxLX_Driver = new JoystickAxis(driver, Constants.ControlConstants.XBOX_AXIS_LEFT_X);
  static JoystickAxis xboxRX_Driver = new JoystickAxis(driver, Constants.ControlConstants.XBOX_AXIS_RIGHT_X);
  static JoystickAxis xboxLY_Driver = new JoystickAxis(driver, Constants.ControlConstants.XBOX_AXIS_LEFT_Y);
  static JoystickAxis xboxRY_Driver = new JoystickAxis(driver, Constants.ControlConstants.XBOX_AXIS_RIGHT_Y);

   //Axis (Operator)
  static JoystickAxis xboxRT_Operator = new JoystickAxis(operator, Constants.ControlConstants.XBOX_AXIS_RIGHT_TRIGGER);
  static JoystickAxis xboxLT_Operator = new JoystickAxis(operator, Constants.ControlConstants.XBOX_AXIS_LEFT_TRIGGER);
  static JoystickAxis xboxLX_Operator = new JoystickAxis(operator, Constants.ControlConstants.XBOX_AXIS_RIGHT_X);
  static JoystickAxis xboxRX_Operator= new JoystickAxis(operator, Constants.ControlConstants.XBOX_AXIS_LEFT_X);
  static JoystickAxis xboxLY_Operator = new JoystickAxis(operator, Constants.ControlConstants.XBOX_AXIS_RIGHT_Y);
  static JoystickAxis xboxRY_Operator = new JoystickAxis(operator, Constants.ControlConstants.XBOX_AXIS_LEFT_Y);


   public static JoystickButton xboxButton(XboxController controller, String button) {
    JoystickButton output;
    if(controller == driver) {
      switch (button.toUpperCase()) {
        case "A":
          output = xboxA_Driver;
          break;
        case "B":
          output = xboxB_Driver;
          break;
        case "X":
          output = xboxX_Driver;
          break;
        case "Y":
          output = xboxY_Driver;
          break;
        case "LS":
          output = xboxL3_Driver;
          break;
        case "RS":
          output = xboxR3_Driver;
          break;
        case "START":
          output = xboxStart_Driver;
          break;
        case "BACK":
          output = xboxBack_Driver;
          break;
        default:
          output = xboxA_Driver;  
      }
    }
    else
    {
      switch (button.toUpperCase()) {
        case "A":
          output = xboxA_Operator;
          break;
        case "B":
          output = xboxB_Operator;
          break;
        case "X":
          output = xboxX_Operator;
          break;
        case "Y":
          output = xboxY_Operator;
          break;
        case "LS":
          output = xboxL3_Operator;
          break;
        case "RS":
          output = xboxL3_Operator;
          break;
        case "START":
          output = xboxStart_Operator;
          break;
        case "BACK":
          output = xboxBack_Operator;
          break;
        default:
          output = xboxBack_Operator;  
      }
    }
    return output;
  }

  public static JoystickAxis xboxAxis(XboxController controller, String axis) {
    JoystickAxis output;
    if(controller == driver) {
      switch (axis.toUpperCase()) {
        case "LT":
          output = xboxLT_Driver;
          break;
        case "RT":
          output = xboxRT_Driver;
          break;
        case "LS-X":
          output = xboxLX_Driver;
          break;
        case "LS-Y":
          output = xboxLY_Driver;
          break;
        case "RS-X":
          output = xboxRX_Driver;
          break;
        case "RS-Y":
          output = xboxRY_Driver;
        default:
          output = xboxRX_Driver;  
      }
    }
    else
    {
      switch (axis.toUpperCase()) {
        case "LT":
          output = xboxLT_Operator;
          break;
        case "RT":
          output = xboxRT_Operator;
          break;
        case "LS-X":
          output = xboxLX_Operator;
          break;
        case "LS-Y":
          output = xboxLY_Operator;
          break;
        case "RS-X":
          output = xboxRX_Operator;
          break;
        case "RS-Y":
          output = xboxRY_Operator;
        default:
          output = xboxRX_Operator;    
      }
    }
    return output;
  }

  public static int xboxDPad(XboxController controller) {
    return controller.getPOV();
  }
}
  
  