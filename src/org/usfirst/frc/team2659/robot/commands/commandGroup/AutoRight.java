package org.usfirst.frc.team2659.robot.commands.commandGroup;

import org.usfirst.frc.team2659.robot.commands.ScoreGear;
import org.usfirst.frc.team2659.robot.commands.driveForward;
import org.usfirst.frc.team2659.robot.commands.driveTo;
import org.usfirst.frc.team2659.robot.commands.turnTo;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoRight extends CommandGroup {
	public AutoRight() {
		addSequential(new driveForward(60));
		addSequential(new turnTo(60, 2));
		addSequential(new driveTo(75, 0.6));
		addSequential(new ScoreGear());
	}
}
