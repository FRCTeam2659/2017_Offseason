package org.usfirst.frc.team2659.robot.subsystems;

import org.usfirst.frc.team2659.robot.RobotMap;


import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	PWMSpeedController SC = RobotMap.climberSC;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    }
    
    public void climbup() {
    		SC.set(1);
    }
    
    public void stop() {
    		SC.set(0);
    }
}

