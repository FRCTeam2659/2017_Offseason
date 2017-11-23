package org.usfirst.frc.team2659.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.cscore.CvSink;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team2659.robot.RobotMap;
import org.usfirst.frc.team2659.robot.commands.drive;
import org.usfirst.frc.team2659.robot.util.GripPipeline;
import org.usfirst.frc.team2659.robot.util.SCWrapper;
import org.usfirst.frc.team2659.robot.util.dummyPIDOutput;
import org.usfirst.frc.team2659.robot.util.encoderWrapper;
import org.usfirst.frc.team2659.robot.util.rampRate;
import org.usfirst.frc.team2659.robot.util.warriorPID;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PowerDistributionPanel;
/**
 *
 */
public class Drivetrain extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	RobotDrive myDrive = RobotMap.myRobot;
	SCWrapper drivetrainLeft = RobotMap.drivetrainLeft;
	SCWrapper drivetrainRight = RobotMap.drivetrainRight;
	DoubleSolenoid cylinder = RobotMap.shiftCylinder;
	Encoder leftEncoder = RobotMap.leftEncoder;
	Encoder rightEncoder = RobotMap.rightEncoder;
	encoderWrapper leftRateEncoder = RobotMap.leftRateEncoder;
	encoderWrapper rightRateEncoder = RobotMap.rightRateEncoder;
	ADXRS450_Gyro gyro = RobotMap.gyro;
	PWMSpeedController SC = RobotMap.intakeSC;
	DoubleSolenoid intakeCylinder = RobotMap.intakeCylinder;
	//PowerDistributionPanel pdp = new PowerDistributionPanel();
	warriorPID leftDisPID = new warriorPID();
	warriorPID rightDisPID = new warriorPID();
	warriorPID leftRotatePID = new warriorPID();
	warriorPID rightRotatePID = new warriorPID();
	warriorPID leftVelPID = new warriorPID();
	warriorPID rightVelPID = new warriorPID();
	warriorPID rotatePID = new warriorPID();
	rampRate leftVelRamp;
	rampRate rightVelRamp;
	dummyPIDOutput leftdummy = new dummyPIDOutput();
	dummyPIDOutput rightdummy = new dummyPIDOutput();
	dummyPIDOutput rotatedummy = new dummyPIDOutput();
	
	GripPipeline vision = new GripPipeline();
	
	public Drivetrain() {
		leftVelRamp = new rampRate(1, 10);
		rightVelRamp = new rampRate(1, 10);
		leftDisPID.setPID(0.025, 0.0025, 0.05);//0.025, 0.00006
		leftDisPID.setSources(leftEncoder);
		leftDisPID.setOutputs(drivetrainLeft);
		rightDisPID.setPID(0.025, 0.0025, 0.05);
		rightDisPID.setSources(rightEncoder);
		rightDisPID.setOutputs(drivetrainRight);
		
		leftRotatePID.setPID(0.02, 0.0, 0.0006);
		leftRotatePID.setSources(gyro);
		leftRotatePID.setOutputs(drivetrainLeft);
		leftRotatePID.setContinuous(false);
		rightRotatePID.setPID(0.02, 0.0, 0.0006);
		rightRotatePID.setSources(gyro);
		rightRotatePID.setOutputs(drivetrainRight);
		rightRotatePID.setContinuous(false);
		
		leftVelPID.setPID(0.15, 0.012, 0.0);
		leftVelPID.setSources(leftRateEncoder);
		leftVelPID.setOutputs(drivetrainLeft);
		leftVelPID.setOutputRange(-0.4, 0.4);
		rightVelPID.setPID(0.15, 0.012, 0.0);
		rightVelPID.setSources(rightRateEncoder);
		rightVelPID.setOutputs(drivetrainRight);
		rightVelPID.setOutputRange(-0.4, 0.4);
		
		rotatePID.setPID(2, 0, 0.02);
		rotatePID.setSources(gyro);
		rotatePID.setOutputs(rotatedummy);
		rotatePID.setOutputRange(-10, 10);
		rotatePID.setInputRange(-180, 180);
		rotatePID.setContinuous(true);
	}
	
	Timer t = new Timer();
	private final double critical = 3.15 * 3.141 / 256;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new drive());
	}
	
    public double aim() {
    		
	    	CvSink sink = CameraServer.getInstance().getVideo(RobotMap.boilerCamera);
	    	Mat frame = new Mat();
	    	sink.grabFrameNoTimeout(frame);
	    	vision.process(frame);
	    	ArrayList<MatOfPoint> result = new ArrayList<MatOfPoint>();
	    	result.addAll(vision.filterContoursOutput());
	    	
	    	double centerX, centerY;
	    	ArrayList<Point> center = new ArrayList<Point>();
	    	if (result.isEmpty()) {
	    		return 1;
	    	}
	    	else {
	    		for (int i = 0; i < result.size(); i++) {
	    			Rect r = Imgproc.boundingRect(result.get(i));
	    			centerX = r.x + r.width / 2;
	    			centerY = r.y + r.height / 2;
	    			center.add(new Point(centerX, centerY));
	    		}
	    	}
	    SmartDashboard.putNumber("goal_center", center.get(0).x);
	    	
	    	centerX = centerY = 0;
	    	for (int i = 0; i < center.size(); i++) {
	    		centerX += center.get(i).x;
	    		centerY += center.get(i).y;
	    	}
	    	centerX /= center.size();
	    	centerY /= center.size();
	    	
	    	double angle;
	    	if (centerX <= 320) {
	    		angle = Math.atan((320-centerX)/320*Math.tan(Math.PI/6))/Math.PI*180;
	    		SmartDashboard.putNumber("angle", -angle);
	    		return -angle;
	    	}
	    	else {
	    		angle = Math.atan((centerX-320)/320*Math.tan(Math.PI/6))/Math.PI*180;
	    		angle += 1.5;
	    		SmartDashboard.putNumber("angle", angle);
	    		return angle;
	    	}
    }
	
	public void curveDrive(double distance, double angle) {

		leftDisPID.setOutputs(leftdummy);
		leftDisPID.setOutputRange(-15, 15);
		leftDisPID.setSetpoint(distance);
		
		leftDisPID.enable();
		
		rotatePID.enable();
	
			double percentDone = (leftEncoder.getDistance() + rightEncoder.getDistance())/2/(distance);
			if (percentDone > 1) {
				percentDone = 1;
			} else if (percentDone < 0) {
				percentDone = 0;
			}
			rotatePID.setSetpoint(angle * percentDone);
			
			double leftVelocity = leftdummy.getOutput() + rotatedummy.getOutput();
			double rightVelocity = leftdummy.getOutput() - rotatedummy.getOutput();
			leftVelocity = leftVelRamp.getNextValue(leftVelocity);
			rightVelocity = rightVelRamp.getNextValue(rightVelocity);
			setVelocity(leftVelocity, rightVelocity);
			
	}
	
	public void curve1Drive(double distance, double r) {
		rightDisPID.setOutputs(rightdummy);
		rightDisPID.setOutputRange(-1, 1);
		rightDisPID.setSetpoint(-distance);
		rightDisPID.enable();
		double curve = Math.pow(Math.E, -r/RobotMap.ROBOT_WIDTH);
		myDrive.drive(-rightdummy.getOutput(), curve);
	}
	
	public void setVelocity(double left, double right) {
		rightVelPID.enable();
		leftVelPID.enable();
		leftVelPID.setSetpoint(left);
		rightVelPID.setSetpoint(-right);
		
	}
	
	public void driveForwardDistance(double distance) {
		leftEncoder.reset();
		rightEncoder.reset();
		
		leftDisPID.setSetpoint(distance);
	    	rightDisPID.setSetpoint(-distance);

	    	leftDisPID.enable();
	    	rightDisPID.enable();
		
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
	
	public void rotate(double degrees) {
		//t.reset();
		//t.start();
		gyro.reset();
		leftRotatePID.setSetpoint(degrees);
		rightRotatePID.setSetpoint(degrees);
		leftRotatePID.enable();
		rightRotatePID.enable();
	    	
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
		leftDisPID.disable();
		rightDisPID.disable();
		leftVelPID.disable();
		rightVelPID.disable();
		leftRotatePID.disable();
		rightRotatePID.disable();
		rotatePID.disable();
		myDrive.drive(0,0);
	}
}
