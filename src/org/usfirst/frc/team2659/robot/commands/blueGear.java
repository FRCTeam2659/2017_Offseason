package org.usfirst.frc.team2659.robot.commands;

import org.usfirst.frc.team2659.robot.Robot;
import org.usfirst.frc.team2659.robot.RobotStateEstimator;
import org.usfirst.frc.team2659.robot.util.Path;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class blueGear extends Command {
	private Path mPath;
    public blueGear(Path path) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drivetrain);
        mPath = path;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.drivetrain.setWantDrivePath(mPath, false);
		RobotStateEstimator.getInstance().onStart(Timer.getFPGATimestamp());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		RobotStateEstimator.getInstance().onLoop(Timer.getFPGATimestamp());
    		Robot.drivetrain.updatePathFollower(Timer.getFPGATimestamp(), 1);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.drivetrain.isDoneWithPath();
        
    }

    // Called once after isFinished returns true
    protected void end() {
    		Robot.drivetrain.stop();
    		SmartDashboard.putBoolean("first", true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		
    }

}
