package org.usfirst.frc.team2659.robot.commands;

import org.usfirst.frc.team2659.robot.Robot;
import org.usfirst.frc.team2659.robot.RobotState;
import org.usfirst.frc.team2659.robot.util.PathAdapter;
import org.usfirst.frc.team2659.robot.util.RigidTransform2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class ResetPose extends Command {
    public ResetPose() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		RigidTransform2d startPose = PathAdapter.getBlueStartPose();
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), startPose);
        Robot.drivetrain.setGyroAngle();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    		
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		
    }

}
