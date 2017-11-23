package org.usfirst.frc.team2659.robot.commands;
import org.usfirst.frc.team2659.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoLeft extends Command {

    public AutoLeft() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drivetrain);
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    
    	Robot.drivetrain.shiftLow();
    	//Robot.drivetrain.setVelocity(20, 30);
    setTimeout(4);
    Robot.drivetrain.curveDrive(100, 0);
    /*	Robot.drivetrain.driveForwardDistance(71);
    	Robot.drivetrain.rotate(-60);
    	Robot.drivetrain.driveForwardDistance(86);
    	Timer t = new Timer();
    	t.start();
    	Timer.delay(0.4);
    	while (t.get() < 2) {
    		Robot.intake.scoreGearAuto();
    	}
    	Robot.intake.stop();
    	Robot.drivetrain.stop();*/
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		//Robot.drivetrain.curve1Drive(97, 80);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
	    	Timer t = new Timer();
	    	t.start();
	    	Timer.delay(0.4);
	    	while (t.get() < 2) {
	    		Robot.intake.scoreGearAuto();
	    	}
    
	    	Robot.intake.stop();
	    	Robot.drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
