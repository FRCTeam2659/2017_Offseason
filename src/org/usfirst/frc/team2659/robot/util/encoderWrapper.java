package org.usfirst.frc.team2659.robot.util;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class encoderWrapper implements PIDSource {
	private Encoder encoder;
	private PIDSourceType pidSource;
	public encoderWrapper(Encoder encoder, PIDSourceType pidSource) {
		this.encoder = encoder;
		this.pidSource = pidSource;
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		this.pidSource = pidSource;
	}
	@Override
	public PIDSourceType getPIDSourceType() {
		return pidSource;
	}
	@Override
	public double pidGet() {
		if (pidSource == PIDSourceType.kDisplacement) {
			return encoder.getDistance();
		} else {
			return encoder.getRate();
		}
	}
}
