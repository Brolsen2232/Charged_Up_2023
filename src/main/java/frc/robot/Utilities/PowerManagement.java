package frc.robot.Utilities;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;

import java.util.*;

public class PowerManagement {

    private PowerDistribution pdp;
    private double overCurrentTime;
    private boolean overCurrentFlag;
    private double maxCurrent;


    public double getPDPCurrent() { return pdp.getTotalCurrent(); }
    public double getSpecificPDPCurrent(int channel) { return pdp.getCurrent(channel); }
    public double getPDPVoltage() { return pdp.getVoltage(); }
    public double getPDPPower() { return pdp.getTotalPower(); }
    public double getPDPEnergy() { return pdp.getTotalEnergy(); }
    public double getPDPTemperature() { return pdp.getTemperature(); }

    public void staticOverCurrentLimit(Collection<CANSparkMax> drivebaseMotors, double defaultCurrentLimit, double newCurrentLimit){
        
        if (getPDPCurrent() > maxCurrent && Timer.getFPGATimestamp() - overCurrentTime > .1) {
            overCurrentFlag = true;
            overCurrentTime = Timer.getFPGATimestamp();
            double newlimit = getPDPCurrent()-maxCurrent;
            newlimit /= drivebaseMotors.size();
            newlimit = defaultCurrentLimit - newlimit;
            for(CANSparkMax motor : drivebaseMotors){
                motor.setSmartCurrentLimit((int)newlimit);
            
            }
          }
          else if (Timer.getFPGATimestamp() - overCurrentTime > 1 && overCurrentFlag){
            overCurrentFlag = false;

            for(CANSparkMax motor : drivebaseMotors){
                motor.setSmartCurrentLimit((int)defaultCurrentLimit);
            
            }
          }
        }
    public void dynamicOverCurrentLimit(){

    }


}