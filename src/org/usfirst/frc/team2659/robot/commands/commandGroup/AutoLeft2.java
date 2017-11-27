package org.usfirst.frc.team2659.robot.commands.commandGroup;

import org.usfirst.frc.team2659.robot.commands.ScoreGear;
import org.usfirst.frc.team2659.robot.commands.curveVelDrive;
import org.usfirst.frc.team2659.robot.commands.driveTo;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoLeft2 extends CommandGroup {
	public AutoLeft2() {
		addSequential(new curveVelDrive(75, 75, 35, 0.25, false));
		addSequential(new curveVelDrive(40, 75, 55, 0.25, true));
		addSequential(new driveTo(25, 0.5));
		//addSequential(new ScoreGear());
	}
}
