package org.usfirst.frc.team2659.robot.util;

import java.util.ArrayList;

import org.usfirst.frc.team2659.robot.util.PathBuilder.Waypoint;

public class BoilerGear implements PathContainer {
	 @Override
	    public Path buildPath() {
	        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
	        sWaypoints.add(new Waypoint(100,90,0,0));
	        sWaypoints.add(new Waypoint(85,70,10,50));
	        sWaypoints.add(new Waypoint(150,40,25,50));
	        sWaypoints.add(new Waypoint(90,25,0,50));

	        return PathBuilder.buildPathFromWaypoints(sWaypoints);
	        //return PathAdapter.getBlueGearPath();
	    }
	    
	    @Override
	    public RigidTransform2d getStartPose() {
	    		return new RigidTransform2d(new Translation2d(100, 100), Rotation2d.fromDegrees(0.0)); 
	    }

	    @Override
	    public boolean isReversed() {
	        return true; 
	    }
}
