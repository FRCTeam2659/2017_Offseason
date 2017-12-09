package org.usfirst.frc.team2659.robot.commands.commandGroup;

import org.usfirst.frc.team2659.robot.commands.DrivePath;
import org.usfirst.frc.team2659.robot.commands.ResetPose;
import org.usfirst.frc.team2659.robot.paths.BoilerGear;
import org.usfirst.frc.team2659.robot.paths.Hopper;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class BlueGearHopper extends CommandGroup {
	public BlueGearHopper() {
		addSequential(new ResetPose());
		Hopper first = new Hopper();
		BoilerGear second = new BoilerGear();
		addSequential(new DrivePath(first.buildPath(), first.isReversed()));
		addSequential(new DrivePath(second.buildPath(), second.isReversed()));
	}
}
