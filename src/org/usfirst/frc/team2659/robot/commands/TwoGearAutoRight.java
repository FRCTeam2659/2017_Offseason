package org.usfirst.frc.team2659.robot.commands;
import org.usfirst.frc.team2659.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TwoGearAutoRight extends Command {

    public TwoGearAutoRight() {
        // Use requires() here to declare subsystem dependencies
       requires(Robot.drivetrain);
       requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
	    	Robot.drivetrain.shiftLow();
	    	Robot.drivetrain.driveForwardDistance(82);
	    	Timer t = new Timer();
	    	t.start();
	    	Timer.delay(0.3);
	    	while (t.get() < 1.1) {
	    		Robot.intake.outtake();
	    	}
	    	Robot.intake.stop();
	    	Robot.drivetrain.driveBackwardDistance(65);
	    	Robot.drivetrain.rotate(90);
	    	Robot.drivetrain.autoIntakeShuffle();
	    	Robot.drivetrain.rotate(-90);
	    	Robot.drivetrain.driveForwardDistance(85);
	    	Timer.delay(0.3);
	    	t.reset();
	    	while (t.get() < 1.5) {
	    		Robot.intake.scoreGearAuto();
	    	}
	    	Robot.intake.stop();
	    	Robot.drivetrain.stop();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	    	
    }
    	
    
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.intake.stop();
    	Robot.drivetrain.stop();
    }
}
