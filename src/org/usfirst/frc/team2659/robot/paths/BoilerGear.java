package org.usfirst.frc.team2659.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team2659.robot.util.control.PathBuilder.Waypoint;
import org.usfirst.frc.team2659.robot.util.control.Path;
import org.usfirst.frc.team2659.robot.util.control.PathBuilder;
import org.usfirst.frc.team2659.robot.util.math.RigidTransform2d;
import org.usfirst.frc.team2659.robot.util.math.Rotation2d;
import org.usfirst.frc.team2659.robot.util.math.Translation2d;

public class BoilerGear {
	    public Path buildPath() {
	        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
	        //sWaypoints.add(new Waypoint(100,90,0,0));
	        //sWaypoints.add(new Waypoint(85,70,10,50));
	        //sWaypoints.add(new Waypoint(150,40,30,50));
	        //sWaypoints.add(new Waypoint(90,25,0,50));
	        sWaypoints.add(new Waypoint(130,130,0,0));
	        sWaypoints.add(new Waypoint(110,90,20,50));
	        sWaypoints.add(new Waypoint(170,30,40,50));
	        sWaypoints.add(new Waypoint(100,20,25,50));
	        sWaypoints.add(new Waypoint(70,90,30,50));
	        sWaypoints.add(new Waypoint(16,90,0,50));

	        return PathBuilder.buildPathFromWaypoints(sWaypoints);
	        //return PathAdapter.getBlueGearPath();
	    }

	    public static RigidTransform2d getStartPose() {
	    		return new RigidTransform2d(new Translation2d(130, 130), Rotation2d.fromDegrees(0.0)); 
	    }

	    public boolean isReversed() {
	        return true; 
	    }
}
