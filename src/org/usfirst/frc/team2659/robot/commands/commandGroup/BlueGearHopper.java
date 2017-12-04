package org.usfirst.frc.team2659.robot.commands.commandGroup;

import org.usfirst.frc.team2659.robot.commands.ResetPose;
import org.usfirst.frc.team2659.robot.commands.blueGear;
import org.usfirst.frc.team2659.robot.commands.blueHopper;
import org.usfirst.frc.team2659.robot.util.BoilerGear;
import org.usfirst.frc.team2659.robot.util.Hopper;
import org.usfirst.frc.team2659.robot.util.PathAdapter;
import org.usfirst.frc.team2659.robot.util.PathContainer;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BlueGearHopper extends CommandGroup {
	public BlueGearHopper() {
		addSequential(new ResetPose());
		PathContainer first = new Hopper();
		addSequential(new blueGear(first.buildPath()));
		//addSequential(new blueGear(PathAdapter.getBlueGearPath()));
		
		PathContainer second = new BoilerGear();
		addSequential(new blueHopper(second.buildPath()));
		//addSequential(new blueHopper(PathAdapter.getBlueHopperPath()));
		
	}
}
