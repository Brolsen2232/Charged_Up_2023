package frc.robot.Utilities.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Axis extends Trigger{

    public Axis whenHeld(final Command command, boolean interruptible) {
        whileActiveContinuous(command);
        return this;
    }

    public Axis whenHeld(final Command command) {
        whileActiveContinuous(command);
        return this;
    }

    public Axis whenReleased(final Command command, boolean interruptible){
        whenReleased(command, interruptible);
        return this;
    }

    public Axis whenReleased(final Command command){
        whenReleased(command);
        return this;
    }
    
}
