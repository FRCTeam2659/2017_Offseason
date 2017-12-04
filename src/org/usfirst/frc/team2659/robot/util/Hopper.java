package org.usfirst.frc.team2659.robot.util;

import java.util.ArrayList;

import org.usfirst.frc.team2659.robot.util.PathBuilder.Waypoint;

public class Hopper implements PathContainer {
	 @Override
	    public Path buildPath() {
	        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
	        sWaypoints.add(new Waypoint(0,50,0,0));
	        sWaypoints.add(new Waypoint(70,50,40,70));
	        sWaypoints.add(new Waypoint(100,90,0,70));

	        return PathBuilder.buildPathFromWaypoints(sWaypoints);
	        //return PathAdapter.getBlueGearPath();
	    }
	    
	    @Override
	    public RigidTransform2d getStartPose() {
	    		//return new RigidTransform2d(PathAdapter.getBlueGearPosition(), Rotation2d.fromDegrees(0.0)); 
	    		return new RigidTransform2d(new Translation2d(0, 50), Rotation2d.fromDegrees(0.0)); 
	    }

	    @Override
	    public boolean isReversed() {
	        return false; 
	    }
}