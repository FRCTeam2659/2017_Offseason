package org.usfirst.frc.team2659.robot.commands;
import org.usfirst.frc.team2659.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoStraight extends Command {

    public AutoStraight() {
        // Use requires() here to declare subsystem dependencies
       requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    Robot.drivetrain.shiftLow();
    	Robot.drivetrain.driveForwardDistance(81);
    	Timer.delay(4);
    	Robot.drivetrain.stop();
    	Timer t = new Timer();
    	t.start();
    	while (t.get() < 1.5) {
    		Robot.intake.scoreGearAuto();
    	}
    	Robot.intake.stop();
    	Robot.drivetrain.stop();
    	setTimeout(3);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    }
    	
    
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
