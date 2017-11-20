package org.usfirst.frc.team2659.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import org.usfirst.frc.team2659.robot.RobotMap;
import org.usfirst.frc.team2659.robot.commands.drive;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj.PowerDistributionPanel;
/**
 *
 */
public class Drivetrain extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	RobotDrive myDrive = RobotMap.myRobot;
	CANTalon leftFrontSC = RobotMap.leftFrontSC;
	CANTalon leftRearSC = RobotMap.leftRearSC;
	CANTalon rightFrontSC = RobotMap.rightFrontSC;
	CANTalon rightRearSC = RobotMap.rightRearSC;
	DoubleSolenoid cylinder = RobotMap.shiftCylinder;
	Encoder leftEncoder = RobotMap.leftEncoder;
	Encoder rightEncoder = RobotMap.rightEncoder;
	ADXRS450_Gyro gyro = RobotMap.gyro;
	PWMSpeedController SC = RobotMap.intakeSC;
	DoubleSolenoid intakeCylinder = RobotMap.intakeCylinder;
	//PowerDistributionPanel pdp = new PowerDistributionPanel();
	PIDController leftFrontPID;
	PIDController leftRearPID;
	PIDController rightFrontPID;
	PIDController rightRearPID;
	
	Timer t = new Timer();
	private final double critical = 3.2 * 3.141 / 256;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new drive());
	}
	
	public void driveForwardDistance(int distance) {
		leftFrontPID = new PIDController(0.025, 0.00005, 0.0, leftEncoder, leftFrontSC);
		leftRearPID = new PIDController(0.025, 0.00005, 0.0, leftEncoder, leftRearSC);
		rightFrontPID = new PIDController(0.025, 0.00005, 0.0, rightEncoder, rightFrontSC);
		rightRearPID = new PIDController(0.025, 0.00005, 0.0, rightEncoder, rightRearSC);
		
		leftEncoder.reset();
		rightEncoder.reset();
		
		leftFrontPID.setOutputRange(-1.0, 1.0);
		leftRearPID.setOutputRange(-1.0, 1.0);
		rightFrontPID.setOutputRange(-1.0, 1.0);
		rightRearPID.setOutputRange(-1.0, 1.0);
		leftFrontPID.setSetpoint(distance);
    	leftRearPID.setSetpoint(distance);
    	rightFrontPID.setSetpoint(distance);
    	rightRearPID.setSetpoint(distance);
    	rightFrontPID.enable();
    	rightRearPID.enable();
    	leftFrontPID.enable();
    	leftRearPID.enable();
		
		/*gyro.reset();
		t.reset();
		t.start();	
		
		boolean i = true;
		while (i) 
		{		
			double leftEncoderCount = leftEncoder.get();
			double rightEncoderCount = rightEncoder.get();
			double leftEncoderDistance = leftEncoderCount * critical; // Distance in Inches	
			double rightEncoderDistance = -rightEncoderCount * critical;
			
			if (leftEncoderDistance < distance && rightEncoderDistance < distance && (gyro.getAngle() <= 1.5 && gyro.getAngle() >= -1.5))
			{
				myDrive.drive(-0.5, 0);
			}
			else if (leftEncoderDistance < distance && rightEncoderDistance < distance && gyro.getAngle() > 1) {
				myDrive.drive(-0.5, -0.1);
			}
			else if (leftEncoderDistance < distance && rightEncoderDistance < distance && gyro.getAngle() < -1) {
				myDrive.drive(-0.5, 0.1);
			}
			else if (t.get() > 6) {
				i = false;
			}
			else {
				i = false;
			}		
		}
		myDrive.drive(0, 0);
		t.stop();*/
	}
	public void driveBackwardDistance(int distance) {
		
		leftEncoder.reset();
		rightEncoder.reset();
		gyro.reset();
		t.reset();
		t.start();
		// Distance in Inches		
		
		boolean i = true;
		while (i) 
		{		
			double leftEncoderCount = leftEncoder.get();
			double rightEncoderCount = rightEncoder.get();
			double leftEncoderDistance = -leftEncoderCount * critical;
			double rightEncoderDistance = rightEncoderCount * critical;

			if (leftEncoderDistance < distance && rightEncoderDistance < distance && (gyro.getAngle() <= 1 && gyro.getAngle() >= -1))
			{
				myDrive.drive(0.8, 0);
			}
			else if (leftEncoderDistance < distance && rightEncoderDistance < distance && gyro.getAngle() > 1) {
				myDrive.drive(0.8, 0.1);
			}
			else if (leftEncoderDistance < distance && rightEncoderDistance < distance && gyro.getAngle() < -1) {
				myDrive.drive(0.8, -0.1);
			}
			else if (t.get() > 4) {
				i = false;
			}
			else {
				i = false;
			}
		}
		
		t.stop();
		myDrive.drive(0, 0);
	}

	public void autoIntakeShuffle() {
		
		leftEncoder.reset();
		rightEncoder.reset();
		gyro.reset();
		t.reset();
		t.start();
	
		intakeCylinder.set(DoubleSolenoid.Value.kReverse);
		SC.set(-1);
		
		boolean i = true;
		while (i) 
		{		
			if (RobotMap.gearSensor.getVoltage() < 0.5) {
				if (gyro.getAngle() <= 1 && gyro.getAngle() >= -1)
				{
					myDrive.drive(-0.6, 0);
				}
				else if (gyro.getAngle() > 1) {
					myDrive.drive(-0.6, -0.1);
				}
				else if (gyro.getAngle() < -1) {
					myDrive.drive(-0.6, 0.1);
				}
				else if (t.get() > 4) {
					SC.set(0);
					intakeCylinder.set(DoubleSolenoid.Value.kForward);
					myDrive.drive(0, 0);
					i = false;
				}
			}
			else if (RobotMap.gearSensor.getVoltage() >= 0.5) {
				intakeCylinder.set(DoubleSolenoid.Value.kForward);
				SC.set(0);
				if (leftEncoder.get() >= 0 && rightEncoder.get() >= 0 && gyro.getAngle() <= 1 && gyro.getAngle() >= -1)
				{
					myDrive.drive(0.8, 0);
				}
				else if (leftEncoder.get() >= 0 && rightEncoder.get() >= 0 && gyro.getAngle() > 1) {
					myDrive.drive(0.8, 0.1);
				}
				else if (leftEncoder.get() >= 0 && rightEncoder.get() >= 0 && gyro.getAngle() < -1) {
					myDrive.drive(0.8, -0.1);
				}
				else if (t.get() > 6) { //the time has to be greater than the first segment!!!
					i = false;
				}
			}		
		}
		
		myDrive.drive(0, 0);
		t.stop();
	}
	
	//Don't delete the code under!
	public void driveBackward() { 
		myDrive.drive(0.5, 0);
	}
	
	public void rotate(int degrees) {
		//t.reset();
		//t.start();
		leftFrontPID = new PIDController(0.02, 0.0, 0.0, gyro, leftFrontSC); //.00005
		leftRearPID = new PIDController(0.02, 0.0, 0.0, gyro, leftRearSC);
		rightFrontPID = new PIDController(0.02, 0.0, 0.0, gyro, rightFrontSC);
		rightRearPID = new PIDController(0.02, 0.0, 0.0, gyro, rightRearSC);
		gyro.reset();
		leftFrontPID.setOutputRange(-1.0, 1.0);
		leftRearPID.setOutputRange(-1.0, 1.0);
		rightFrontPID.setOutputRange(-1.0, 1.0);
		rightRearPID.setOutputRange(-1.0, 1.0);
		leftFrontPID.setContinuous(false);
		leftRearPID.setContinuous(false);
		rightFrontPID.setContinuous(false);
		rightRearPID.setContinuous(false);
		leftFrontPID.setSetpoint(degrees);
    	leftRearPID.setSetpoint(degrees);
    	rightFrontPID.setSetpoint(degrees);
    	rightRearPID.setSetpoint(degrees);
    	rightFrontPID.enable();
    	rightRearPID.enable();
    	leftFrontPID.enable();
    	leftRearPID.enable();
    	
		/*double status = gyro.getAngle();
		double goal = degrees + status;
		if (degrees > 0) {
			boolean i = true;
			while (gyro.getAngle() < goal && i)
			{
				myDrive.tankDrive(-0.5, 0.5); //turn right 0.7 old value
				if (t.get() > 4) {
					i = false;
				}
			}
		}
		else {
			boolean i = true;
			while (gyro.getAngle() > goal && i) {
				myDrive.tankDrive(0.5, -0.5); //turn left
				if (t.get() > 4) {
					i = false;
				}
			}
		}
		myDrive.drive(0, 0);
		t.stop();*/
	}
	
	public void shiftHigh() {
		cylinder.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void shiftLow() {
		cylinder.set(DoubleSolenoid.Value.kForward);
	}
	
	public void warriorDrive(double y, double z) {
		myDrive.arcadeDrive(-y, -z);
	}
	
	public void stop() {
		leftFrontPID.disable();
		leftRearPID.disable();
		rightFrontPID.disable();
		rightRearPID.disable();
		myDrive.drive(0,0);
	}
}
