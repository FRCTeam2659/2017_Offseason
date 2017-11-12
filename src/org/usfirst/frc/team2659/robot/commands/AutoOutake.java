package org.usfirst.frc.team2659.robot.commands;
import org.usfirst.frc.team2659.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoOutake extends Command {

    public AutoOutake() {
        // Use requires() here to declare subsystem dependencies
       requires(Robot.drivetrain);
       requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Timer.delay(6.5);
    	Robot.drivetrain.shiftLow();
    	Robot.intake.intakeDown();
    	Robot.drivetrain.driveForwardDistance(32);
    	Timer t = new Timer();
    	t.start();
    	while (t.get() < 0.5) {
    		Robot.intake.outtake();
    	}
    	Robot.drivetrain.driveBackwardDistance(32);
    	Robot.intake.intakeUp();
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
    }
}
