package org.usfirst.frc.team2659.robot.commands;

import org.usfirst.frc.team2659.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ScoreGear extends Command {
	Timer t = new Timer();
    public ScoreGear() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.intake);
        //requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		/*t.start();
    		while(t.get() < 0.2) {
    			Robot.intake.outtake();
    		}
    		t.stop();*/
    		setTimeout(1);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
	    	//Robot.intake.outtake();
	    	//Robot.drivetrain.driveBackward();
	    	Robot.intake.scoreGearAuto();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
	    Robot.intake.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
