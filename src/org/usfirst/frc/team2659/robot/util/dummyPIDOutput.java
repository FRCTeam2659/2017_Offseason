package org.usfirst.frc.team2659.robot.util;

import edu.wpi.first.wpilibj.PIDOutput;

public class dummyPIDOutput implements PIDOutput {
	double output;
    
    public dummyPIDOutput()
    {
        output = 0;
    }

    public void pidWrite(double output) {
        this.output = output;
    }

    public double getOutput() {
        return output;
    }
}
