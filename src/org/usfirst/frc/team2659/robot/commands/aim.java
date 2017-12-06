package org.usfirst.frc.team2659.robot.commands;
import org.usfirst.frc.team2659.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class aim extends Command {
	double angle;
	Command drive = new drive();
    public aim() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drivetrain);
        requires(Robot.climber);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
	    	drive.cancel();
	    	Robot.drivetrain.rotateDistance(Robot.drivetrain.aim());
	    	setTimeout(5);
    		
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		//Robot.drivetrain.rotate(Robot.drivetrain.aim());
    
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    		Robot.drivetrain.stop();
    		drive.start();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    
    }
}
