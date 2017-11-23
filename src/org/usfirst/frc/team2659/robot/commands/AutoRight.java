package org.usfirst.frc.team2659.robot.commands;
import org.usfirst.frc.team2659.robot.Robot;
import org.usfirst.frc.team2659.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoRight extends Command {

    public AutoRight() {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.drivetrain);
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(15);
    Robot.drivetrain.shiftLow();
    Robot.drivetrain.driveForwardDistance(71);
    	Timer.delay(2.5);
    Robot.drivetrain.stop();
    	Robot.drivetrain.rotate(60);
    	Timer.delay(1);
    	Robot.drivetrain.stop();
    Robot.drivetrain.driveForwardDistance(86);
    Timer.delay(2.5);
    Robot.drivetrain.stop();
    	Timer t = new Timer();
    	t.start();
    	while (t.get() < 2) {
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
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.stop();
    	Robot.intake.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
