package org.usfirst.frc.team2659.robot.commands.commandGroup;

import org.usfirst.frc.team2659.robot.commands.ScoreGear;
import org.usfirst.frc.team2659.robot.commands.curve1Drive;
import org.usfirst.frc.team2659.robot.commands.driveTo;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoRight1 extends CommandGroup {
	public AutoRight1() {
		addSequential(new curve1Drive(116, 123, 0.6));
		addSequential(new driveTo(15, 0.5));
		addSequential(new ScoreGear());
	}
}
