package org.usfirst.frc.team2659.robot.util;

public interface PathContainer {
    Path buildPath();

    RigidTransform2d getStartPose();

    boolean isReversed();
}
