package org.usfirst.frc.team2659.robot.util.drivers;

import com.ctre.CANTalon;

public class LazyCANTalon extends CANTalon {
	protected double mLastSet = Double.NaN;
    protected TalonControlMode mLastControlMode = null;

    public LazyCANTalon(int deviceNumber, int controlPeriodMs, int enablePeriodMs) {
        super(deviceNumber, controlPeriodMs, enablePeriodMs);
    }

    public LazyCANTalon(int deviceNumber, int controlPeriodMs) {
        super(deviceNumber, controlPeriodMs);
    }

    public LazyCANTalon(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void set(double value) {
        if (value != mLastSet || getControlMode() != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = getControlMode();
            super.set(value);
        }
    }
}
