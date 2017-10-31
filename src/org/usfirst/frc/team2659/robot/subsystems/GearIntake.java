package org.usfirst.frc.team2659.robot.subsystems;

import org.usfirst.frc.team2659.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GearIntake extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	PWMSpeedController SC = RobotMap.intakeSC;
	DoubleSolenoid cylinder = RobotMap.intakeCylinder;
	AnalogInput sensor = RobotMap.gearSensor;
	RobotDrive myDrive = RobotMap.myRobot;
	//PowerDistributionPanel powerPanel = RobotMap.pdp;
	//public double current;
	public double value;
	//Timer t =  new Timer();
	
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
    public void scoreGearAuto() {
	    	SC.set(0.75);
	    	myDrive.drive(0.5, 0);
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
    }
}

