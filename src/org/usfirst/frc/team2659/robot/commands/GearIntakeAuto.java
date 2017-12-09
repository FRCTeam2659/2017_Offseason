package org.usfirst.frc.team2659.robot.commands;


import org.usfirst.frc.team2659.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;


public class GearIntakeAuto extends Command {
	private boolean isFinished;
    public GearIntakeAuto() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		isFinished = Robot.intake.autoRun();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isFinished;
    }

    // Called once after isFinished returns true
    protected void end() {
    		Timer t = new Timer();
    		t.start();
    		while (t.get() < 1) {
	    		Robot.oi.driveStick.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
			Robot.oi.driveStick.setRumble(GenericHID.RumbleType.kRightRumble, 1);
    		}
    		t.stop();
    		Robot.oi.driveStick.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
		Robot.oi.driveStick.setRumble(GenericHID.RumbleType.kRightRumble, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
