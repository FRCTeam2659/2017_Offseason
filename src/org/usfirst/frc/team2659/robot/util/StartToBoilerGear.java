package org.usfirst.frc.team2659.robot.util;

public class StartToBoilerGear implements PathContainer {

    @Override
    public Path buildPath() {
        return PathAdapter.getRedGearPath();
    }

    @Override
    public RigidTransform2d getStartPose() {
        return PathAdapter.getRedStartPose();
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}
