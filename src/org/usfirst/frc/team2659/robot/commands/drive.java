package org.usfirst.frc.team2659.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team2659.robot.Robot;
import org.usfirst.frc.team2659.robot.util.TortoDriveHelper;

public class drive extends Command {
	private TortoDriveHelper myDrive = new TortoDriveHelper();
	private int isReversed = 1;
	public drive() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		//Robot.drivetrain.warriorDrive(Robot.oi.driveStick.getY(), Robot.oi.driveStick.getX());
		if (Robot.oi.driveReverseButton.get())
			isReversed = -1;
		else if (Robot.oi.driveNormalButton.get())
			isReversed = 1;
		Robot.drivetrain.setOpenLoop(myDrive.tortoDrive(isReversed*Robot.oi.driveStick.getRawAxis(1), Robot.oi.driveStick.getRawAxis(4), !Robot.drivetrain.isHighGear(), Robot.drivetrain.isHighGear()));
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		this.cancel();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		
	}
}
