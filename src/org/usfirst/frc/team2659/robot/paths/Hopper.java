package org.usfirst.frc.team2659.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team2659.robot.util.control.Path;
import org.usfirst.frc.team2659.robot.util.control.PathBuilder;
import org.usfirst.frc.team2659.robot.util.control.PathBuilder.Waypoint;
import org.usfirst.frc.team2659.robot.util.math.RigidTransform2d;
import org.usfirst.frc.team2659.robot.util.math.Rotation2d;
import org.usfirst.frc.team2659.robot.util.math.Translation2d;

public class Hopper {

	    public Path buildPath() {
	        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
	        //sWaypoints.add(new Waypoint(0,50,0,0));
	        //sWaypoints.add(new Waypoint(70,50,40,70));
	        //sWaypoints.add(new Waypoint(100,90,0,70));
	        sWaypoints.add(new Waypoint(16,90,0,0));
	        sWaypoints.add(new Waypoint(110,90,35,65));
	        sWaypoints.add(new Waypoint(130,130,0,65));

	        return PathBuilder.buildPathFromWaypoints(sWaypoints);
	        //return PathAdapter.getBlueGearPath();
	    }
	    

	    public static RigidTransform2d getStartPose() {
	    		//return new RigidTransform2d(PathAdapter.getBlueGearPosition(), Rotation2d.fromDegrees(0.0)); 
	    		return new RigidTransform2d(new Translation2d(16, 90), Rotation2d.fromDegrees(0.0)); 
	    }


	    public boolean isReversed() {
	        return false; 
	    }
}