package org.usfirst.frc.team2659.robot.util;

import java.util.ArrayList;

import org.usfirst.frc.team2659.robot.util.PathBuilder.Waypoint;

public class BoilerGear implements PathContainer {
	 @Override
	    public Path buildPath() {
	        /*ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
	        //sWaypoints.add(new Waypoint(16,90,0,0));
	        sWaypoints.add(new Waypoint(100,90,20,20));
	        sWaypoints.add(new Waypoint(130,110,0,70));*/

	        //return PathBuilder.buildPathFromWaypoints(sWaypoints);
	        return PathAdapter.getBlueGearPath();
	    }
	    
	    @Override
	    public RigidTransform2d getStartPose() {
	        return PathAdapter.getBlueStartPose();
	    		//return new RigidTransform2d(new Translation2d(16, 90), Rotation2d.fromDegrees(0.0)); 
	    }

	    @Override
	    public boolean isReversed() {
	        return false; 
	    }
		// WAYPOINT_DATA: [{"position":{"x":16,"y":90},"speed":0,"radius":0,"comment":""},{"position":{"x":100,"y":90},"speed":60,"radius":0,"comment":""},{"position":{"x":130,"y":120},"speed":20,"radius":0,"comment":""}]
		// IS_REVERSED: false
		// FILE_NAME: BoilerGear
}
