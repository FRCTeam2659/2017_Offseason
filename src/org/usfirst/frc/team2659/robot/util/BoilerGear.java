package org.usfirst.frc.team2659.robot.util;

import java.util.ArrayList;

import org.usfirst.frc.team2659.robot.util.PathBuilder.Waypoint;

public class BoilerGear implements PathContainer {
	 @Override
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
	    
	    @Override
	    public RigidTransform2d getStartPose() {
	    		return new RigidTransform2d(new Translation2d(130, 130), Rotation2d.fromDegrees(0.0)); 
	    }

	    @Override
	    public boolean isReversed() {
	        return true; 
	    }
}
