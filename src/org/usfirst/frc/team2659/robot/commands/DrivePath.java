package org.usfirst.frc.team2659.robot.commands;

import org.usfirst.frc.team2659.robot.Robot;
import org.usfirst.frc.team2659.robot.RobotStateEstimator;
import org.usfirst.frc.team2659.robot.util.control.Path;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class DrivePath extends Command {
	private Path mPath;
	private Boolean mReversed;
    public DrivePath(Path path, boolean reversed) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drivetrain);
        mPath = path;
        mReversed = reversed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.drivetrain.setWantDrivePath(mPath, mReversed);
		RobotStateEstimator.getInstance().onStart(Timer.getFPGATimestamp());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		double timestamp = Timer.getFPGATimestamp();
    		RobotStateEstimator.getInstance().onLoop(timestamp);
    		Robot.drivetrain.updatePathFollower(timestamp);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.drivetrain.isDoneWithPath(); 
    }

    // Called once after isFinished returns true
    protected void end() {
    		Robot.drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		
    }

}
