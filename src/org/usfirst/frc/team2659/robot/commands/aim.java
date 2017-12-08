package org.usfirst.frc.team2659.robot.commands;
import org.usfirst.frc.team2659.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class aim extends Command {
	double angle;
	Command drive = new drive();
    public aim() {
        requires(Robot.drivetrain);
    }

    protected void initialize() {
	    	double angle = Robot.drivetrain.aim();
	    	
	    	if (-.75 < angle && angle < .75)
	    		setTimeout(0.1);
	    	else {
	    		drive.cancel();
	    		Robot.drivetrain.rotateDistance(angle);  
	    		setTimeout(2);
	    	} 		
    }

    protected void execute() {
    		//Robot.drivetrain.rotate(Robot.drivetrain.aim());
    }

    protected boolean isFinished() {
        return isTimedOut();
    }

    protected void end() {
    		Robot.drivetrain.stop();
    		drive.start();
    }

    protected void interrupted() {
    
    }
}
