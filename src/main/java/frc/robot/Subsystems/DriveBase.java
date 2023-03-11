package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.Utilities.PowerManagement;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.*;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import java.util.ArrayList;
import java.util.Collection;

//TODO: Remember to bring gear shifter back
public class DriveBase extends SubsystemBase {
  private AHRS navxGyro;
  
  // Configuring Motors
  private CANSparkMax leftDrive1;
  private CANSparkMax leftDrive2;
  private CANSparkMax leftDrive3;

  private CANSparkMax rightDrive1;
  private CANSparkMax rightDrive2;
  private CANSparkMax rightDrive3;

  private CANSparkMax armMotorLeft, armMotorRight;
  private CANSparkMax transMotor;

  private CANPIDController pidTransMotor, pidRotateMotor;

  private RelativeEncoder rotate_encoder, trans_encoder;

  // Configuring Drives
  private MotorControllerGroup leftDrives;
  private MotorControllerGroup rightDrives;
  private DifferentialDrive ourDrive;
  private DifferentialDriveOdometry odometry;

  private boolean compressorState = true;

  //PID stuff
  //private int loopIndex, slotIndex;
  private double kFF = 1;
  private double kP = 23.183;
  private double kI = 0;//1;
  private double kD = 0.69334;//1.5784;
  private double kMinOutput;
  private double maxVel = 36.211; 
  private double minVel;
  private double maxAcc;
  private double allowedErr = 0.125;

  //private int iaccum = 0;

  // Solenoid
  private Solenoid gearShifter;

  // Sensors
  private RelativeEncoder leftEncoders[];
  private RelativeEncoder rightEncoders[];
  public Compressor compressor;
  private PowerDistribution pdp;
  private PowerManagement powerManagement;
  Collection<CANSparkMax> drivebaseMotors;
  private double overCurrentTime;
  private boolean overCurrentFlag;

  public DriveBase() {

    //Instantating the physical parts on the drivebase
    //compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    

    pdp = new PowerDistribution();
    navxGyro = new AHRS(SPI.Port.kMXP);
  
    //leftDrive1 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_LEFT_1, MotorType.kBrushed);
    leftDrive2 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_LEFT_2, MotorType.kBrushed);
   leftDrive3 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_LEFT_3, MotorType.kBrushed);
    //rightDrive1 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_1, MotorType.kBrushed);
    rightDrive2 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_2, MotorType.kBrushed);
   rightDrive3 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_3, MotorType.kBrushed);
    
    armMotorLeft = new CANSparkMax(Constants.DriveConstants.ARM_MOTOR_LEFT, MotorType.kBrushless);
    armMotorRight = new CANSparkMax(Constants.DriveConstants.ARM_MOTOR_RIGHT, MotorType.kBrushless);

    transMotor = new CANSparkMax(Constants.DriveConstants.TRANS_MOTOR, MotorType.kBrushless);

    pidRotateMotor = armMotorLeft.getPIDController();
    pidTransMotor = transMotor.getPIDController();

    rotate_encoder = armMotorLeft.getEncoder();
    trans_encoder = transMotor.getEncoder();

    pidRotateMotor.setP(0.015642);
    pidRotateMotor.setI(0);
    pidRotateMotor.setD(0.12823);
    pidTransMotor.setP(0.0091642);

    //pidTransMotor.setP(0.015);
    pidTransMotor.setI(0);
    pidTransMotor.setD(0.12823);

    armMotorLeft.setIdleMode(IdleMode.kBrake);
    armMotorRight.setIdleMode(IdleMode.kBrake);
    //armMotorLeft.follow(armMotorRight);
    //armMotorLeft.setInverted(true);
    powerManagement = new PowerManagement();
    leftDrives = new MotorControllerGroup( leftDrive2, leftDrive3);
    rightDrives = new MotorControllerGroup(rightDrive2, rightDrive3);
   // drivebaseMotors.add(leftDrive3);
   // drivebaseMotors.add(rightDrive3);

    ourDrive = new DifferentialDrive(leftDrives, rightDrives);

  }
  public void rotateArm(double armSpeed, boolean run){
    if(run){
      armMotorRight.set(armSpeed);
      armMotorLeft.set(-1*armSpeed);


    }
    else{
      armMotorRight.set(0);
      armMotorLeft.set(0);
    }
  }

  public void translateArm(double armSpeed, boolean run){
    if(run){
      transMotor.set(armSpeed);


    }
    else{
      transMotor.set(0);
    }
  }

  public void pidRotateArm(){
    setPIDPosition(pidRotateMotor, rotate_encoder, ControlType.kPosition,  -43.99 );
    //setPIDPosition(pidRotateMotor, rotate_encoder, ControlType.kPosition,  -40.99 );
  }
  public void pidTranslateArm(){
    setPIDPosition(pidTransMotor, trans_encoder, ControlType.kPosition, 67.05 );
    //setPIDPosition(pidTransMotor, trans_encoder, ControlType.kPosition,  167 );
  }
  public void resetArmEncoderDistance(){
    trans_encoder.setPosition(0);
    rotate_encoder.setPosition(0);
  }
  
  public void initPIDController(CANPIDController m_pidController, double kP, double kI, double kD, double kIz, double kMaxOutput){
    double p = 0, i = 0, d = 0, iz = 0, ff = 0, max = 0, min = 0, maxV = 0 , minV = 0, maxA = 0, allE = 0;
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { m_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { m_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { m_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
  }

  public void setPIDPosition(CANPIDController m_pidController, RelativeEncoder m_encoder,  ControlType kposition, double setPoint){    
      
      m_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
    
  }
  public void printEncoderDistances(){
    System.out.println("Rotational Encoder " + rotate_encoder.getPosition());
    System.out.println("Translational Encoder" + trans_encoder.getPosition());
  }
  public void drive(double left, double right) {
    ourDrive.tankDrive(left, right);
  }
  public double getGyro(){
    return navxGyro.getAngle();
  }
}