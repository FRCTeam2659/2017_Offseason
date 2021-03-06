package org.usfirst.frc.team2659.robot.commands;

import org.usfirst.frc.team2659.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
/**
 *
 */
public class driveForward extends Command {
	private double distance;
	private boolean finished;
    public driveForward(double distance) {
        this.distance = distance;
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		setTimeout(4);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		finished = Robot.drivetrain.driveForwardDistance(distance);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished || isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    		Robot.drivetrain.zeroSensors();
    		Robot.drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		finished = true;
    }
}
