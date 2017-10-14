package org.usfirst.frc.team2659.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import org.usfirst.frc.team2659.robot.RobotMap;
import org.usfirst.frc.team2659.robot.commands.drive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
/**
 *
 */
public class Drivetrain extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	RobotDrive myDrive = RobotMap.myRobot;
	DoubleSolenoid cylinder = RobotMap.shiftCylinder;
	Encoder leftEncoder = RobotMap.leftEncoder;
	Encoder rightEncoder = RobotMap.rightEncoder;
	ADXRS450_Gyro gyro = RobotMap.gyro;
	PowerDistributionPanel pdp = new PowerDistributionPanel();
	Timer t = new Timer();

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new drive());
	}
	
	public void warriorDrive(double y, double z) {
		myDrive.arcadeDrive(y, z);
	}
	
	public void driveForward() {
		myDrive.drive(-0.1, 0);
	}
	
	public void driveForwardDistance(double distance) {
		
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
			//double rightEncoderCount = rightEncoder.get();
			double leftEncoderDistance = (leftEncoderCount / 256) * (2 * 3.14 * (1.589));
			//double rightEncoderDistance = (rightEncoderCount / 256) * (2 * 3.14 * (1.75));
			
			/*double current0 = pdp.getCurrent(0);
			double current1 = pdp.getCurrent(1);
			double current14 = pdp.getCurrent(14);
			double current15 = pdp.getCurrent(15);*/
			
			if (leftEncoderDistance < distance && (gyro.getAngle() <= 1 && gyro.getAngle() >= -1))
			{
				myDrive.drive(-0.5, 0);
			}
			else if (leftEncoderDistance < distance && gyro.getAngle() > 1) {
				myDrive.drive(-0.5, -0.1);
			}
			else if (leftEncoderDistance < distance &&  gyro.getAngle() < -1) {
				myDrive.drive(-0.5, 0.1);
			}
			else if (t.get() > 7) {
				i = false;
			}
			else {
				i = false;
			}
			/*else if (current0 > 15 || current1 > 15 || current14 > 15 || current15 > 15)
			{
				i = false;
			}*/
			
		}
		myDrive.drive(0, 0);
	}
	
	public void driveBackward() {
		myDrive.drive(0.5, 0);
	}
	
	public void rotate(int degrees) {
		t.reset();
		t.start();
		//gyro.reset();
		double status = gyro.getAngle();
		double goal = degrees + status;
		if (degrees > 0) {
			boolean i = true;
			while (gyro.getAngle() < goal && i)
			{
				myDrive.tankDrive(-0.5, 0.5);
				if (t.get() > 3) {
					i = false;
				}
			}
		}
		else {
			boolean i = true;
			while (gyro.getAngle() > goal && i) {
				myDrive.tankDrive(0.5, -0.5);
				if (t.get() > 3) {
					i = false;
				}
			}
		}
		myDrive.drive(0, 0);
	}
	
	public void turnLeft() {
		myDrive.drive(0.75, -1);
	}
	
	public void turnRight() {
		myDrive.drive(0.75, 1);
	}
	public void shiftHigh() {
		cylinder.set(DoubleSolenoid.Value.kReverse);
	}
	public void shiftLow() {
		cylinder.set(DoubleSolenoid.Value.kForward);
	}
	public void shiftDrive(boolean shift) {
		
		if(shift == true) {
			cylinder.set(DoubleSolenoid.Value.kForward);
		}
		else {
			cylinder.set(DoubleSolenoid.Value.kReverse);		
		}
	}
	
	public void stop() {
		myDrive.drive(0,0);
	}
}
