package org.usfirst.frc.team2659.robot.commands.commandGroup;

import org.usfirst.frc.team2659.robot.commands.ScoreGear;
import org.usfirst.frc.team2659.robot.commands.curveVelDrive;
import org.usfirst.frc.team2659.robot.commands.driveTo;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoLeft2 extends CommandGroup {
	public AutoLeft2() {
		addSequential(new curveVelDrive(75, 76, 70, 0.25));
		addSequential(new curveVelDrive(20, 76, 25, 0.25));
		addSequential(new driveTo(50, 0.5));
		addSequential(new ScoreGear());
	}
}
