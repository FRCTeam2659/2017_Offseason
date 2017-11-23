package org.usfirst.frc.team2659.robot.subsystems;

import org.usfirst.frc.team2659.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GearIntake extends Subsystem {

	PWMSpeedController SC = RobotMap.intakeSC;
	DoubleSolenoid cylinder = RobotMap.intakeCylinder;
	AnalogInput sensor = RobotMap.gearSensor;
	RobotDrive myDrive = RobotMap.myRobot;
	//PowerDistributionPanel powerPanel = RobotMap.pdp;
	//public double current;
	private double value;
	private boolean i = true;
	Timer t =  new Timer();
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    }
    public void autoRun() {	
	    	value = sensor.getVoltage();
	    	//current = powerPanel.getCurrent(4);
	    	//t.start();
	    	
	    	if (value < 0.5)  {
		    	cylinder.set(DoubleSolenoid.Value.kReverse);
		    	SC.set(-1);
		    	//t.delay(0.5);
	    	}
	    	/*else if (value < 0.5 && current >= 10)
	    	{
	    		cylinder.set(DoubleSolenoid.Value.kReverse);
	    		SC.set(1);
	    	}*/
	    	else {
	    		SC.set(0);
			cylinder.set(DoubleSolenoid.Value.kForward);
	    	}
    }
    
    public void autoRunReverse() {
	    	cylinder.set(DoubleSolenoid.Value.kReverse);
	    	SC.set(1);
    }
    public void scoreGearAuto() { //all autonomous commands and operator use this method
    	SC.set(1);
    		if (i) {
		    	t.start();
		    	i = false;
    		}
	    	if (t.get() > 0.15 && t.get() < 0.4) {
	    		myDrive.drive(-0.5, 0);
	    		cylinder.set(DoubleSolenoid.Value.kReverse);
	    	}
	    	if (t.get() >= 0.4) {
	    		t.stop();
	    		myDrive.drive(-0.5, 0);
	    		cylinder.set(DoubleSolenoid.Value.kForward);
	    	}
    }
    public void intakeUp() {
    		cylinder.set(DoubleSolenoid.Value.kForward);
    }
    public void intakeDown() {
    		cylinder.set(DoubleSolenoid.Value.kReverse);
    }
    public void intake() {
    		SC.set(-1);
    }
    public void outtake() {
    		SC.set(1);
    }
    public void stop() {
    		SC.set(0);
    		i = true;
    }
}

