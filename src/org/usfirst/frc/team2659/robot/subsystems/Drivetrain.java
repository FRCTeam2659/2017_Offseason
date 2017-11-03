package org.usfirst.frc.team2659.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import org.usfirst.frc.team2659.robot.RobotMap;
import org.usfirst.frc.team2659.robot.commands.drive;
import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
	PWMSpeedController SC = RobotMap.intakeSC;
	DoubleSolenoid intakeCylinder = RobotMap.intakeCylinder;
	//PowerDistributionPanel pdp = new PowerDistributionPanel();
	Timer t = new Timer();

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new drive());
	}
	
	public void warriorDrive(double y, double z) {
		myDrive.arcadeDrive(y, z);
	}
	
	public void driveForwardDistance(int distance) {
		
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
			double leftEncoderDistance = (leftEncoderCount / 256) * (2 * 3.14 * (1.75));
			double rightEncoderDistance = (rightEncoderCount / 256) * (2 * 3.14 * (1.75));
			
			/*double current0 = pdp.getCurrent(0);
			double current1 = pdp.getCurrent(1);
			double current14 = pdp.getCurrent(14);
			double current15 = pdp.getCurrent(15);*/
			
			if (leftEncoderDistance < distance && rightEncoderDistance < distance && (gyro.getAngle() <= 1 && gyro.getAngle() >= -1))
			{
				myDrive.drive(-0.6, 0);
			}
			else if (leftEncoderDistance < distance && rightEncoderDistance < distance && gyro.getAngle() > 1) {
				myDrive.drive(-0.6, -0.1);
			}
			else if (leftEncoderDistance < distance && rightEncoderDistance < distance && gyro.getAngle() < -1) {
				myDrive.drive(-0.6, 0.1);
			}
			else if (t.get() > 5) {
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
			double leftEncoderDistance = (-leftEncoderCount / 256) * (2 * 3.14 * (1.75));
			double rightEncoderDistance = (-rightEncoderCount / 256) * (2 * 3.14 * (1.75));

			if (leftEncoderDistance < distance && rightEncoderDistance < distance && (gyro.getAngle() <= 1 && gyro.getAngle() >= -1))
			{
				myDrive.drive(1, 0);
			}
			else if (leftEncoderDistance < distance && rightEncoderDistance < distance && gyro.getAngle() > 1) {
				myDrive.drive(1, 0.1);
			}
			else if (leftEncoderDistance < distance && rightEncoderDistance < distance && gyro.getAngle() < -1) {
				myDrive.drive(1, -0.1);
			}
			else if (t.get() > 4) {
				i = false;
			}
			else {
				i = false;
			}
		}
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
					i = false;
				}
			}
			else if (RobotMap.gearSensor.getVoltage() >= 0.5) {
				intakeCylinder.set(DoubleSolenoid.Value.kForward);
				SC.set(0);
				if (leftEncoder.get() >= 0 && rightEncoder.get() >= 0 && gyro.getAngle() <= 1 && gyro.getAngle() >= -1)
				{
					myDrive.drive(1, 0);
				}
				else if (leftEncoder.get() >= 0 && rightEncoder.get() >= 0 && gyro.getAngle() > 1) {
					myDrive.drive(1, 0.1);
				}
				else if (leftEncoder.get() >= 0 && rightEncoder.get() >= 0 && gyro.getAngle() < -1) {
					myDrive.drive(1, -0.1);
				}
				else if (t.get() > 3) {
					i = false;
				}
			}
			else {
				i = false;
			}		
		}
		
		SC.set(0);
		intakeCylinder.set(DoubleSolenoid.Value.kForward);
		myDrive.drive(0, 0);
	}
	public void driveBackward() {
		myDrive.drive(0.5, 0);
	}
	
	public void rotate(int degrees) {
		t.reset();
		t.start();
		gyro.reset();
		double status = gyro.getAngle();
		double goal = degrees + status;
		if (degrees > 0) {
			boolean i = true;
			while (gyro.getAngle() < goal && i)
			{
				myDrive.tankDrive(-0.7, 0.7); //turn right
				if (t.get() > 4) {
					i = false;
				}
			}
		}
		else {
			boolean i = true;
			while (gyro.getAngle() > goal && i) {
				myDrive.tankDrive(0.7, -0.7); //turn left
				if (t.get() > 4) {
					i = false;
				}
			}
		}
		myDrive.drive(0, 0);
	}
	
	public void shiftHigh() {
		cylinder.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void shiftLow() {
		cylinder.set(DoubleSolenoid.Value.kForward);
	}
	
	public void stop() {
		myDrive.drive(0,0);
	}
}
