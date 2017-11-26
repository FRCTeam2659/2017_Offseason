package org.usfirst.frc.team2659.robot.commands;

import org.usfirst.frc.team2659.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
/**
 *
 */
public class driveTo extends Command {
	private double distance;
	private double tolerance;
	private boolean finished;
    public driveTo(double distance, double tolerance) {
        this.distance = distance;
    		this.tolerance = tolerance;
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		setTimeout(3);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		finished = Robot.drivetrain.driveTo(distance, tolerance);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished || isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    		Robot.drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		finished = true;
    }
}
