package org.usfirst.frc.team2659.robot.util;

public interface Interpolable<T> {
	public T interpolate(T other, double x);
}
