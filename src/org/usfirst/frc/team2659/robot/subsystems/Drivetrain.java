package org.usfirst.frc.team2659.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
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
import org.usfirst.frc.team2659.robot.Kinematics;
import org.usfirst.frc.team2659.robot.RobotMap;
import org.usfirst.frc.team2659.robot.RobotState;
import org.usfirst.frc.team2659.robot.util.CANTalonFactory;
import org.usfirst.frc.team2659.robot.util.DriveSignal;
import org.usfirst.frc.team2659.robot.util.GripPipeline;
import org.usfirst.frc.team2659.robot.util.Lookahead;
import org.usfirst.frc.team2659.robot.util.Path;
import org.usfirst.frc.team2659.robot.util.PathFollower;
import org.usfirst.frc.team2659.robot.util.RigidTransform2d;
import org.usfirst.frc.team2659.robot.util.SCWrapper;
import org.usfirst.frc.team2659.robot.util.Twist2d;
import org.usfirst.frc.team2659.robot.util.dummyPIDOutput;
import org.usfirst.frc.team2659.robot.util.encoderWrapper;
import org.usfirst.frc.team2659.robot.util.rampRate;
import org.usfirst.frc.team2659.robot.util.warriorPID;

import com.ctre.CANTalon;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.VelocityMeasurementPeriod;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PowerDistributionPanel;


public class Drivetrain extends Subsystem {
	
    private static final int kHighGearVelocityControlSlot = 1;
    
    private Path mCurrentPath = null;
    private RobotState mRobotState = RobotState.getInstance();
    private PathFollower mPathFollower;
	
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
	warriorPID leftDisPID = new warriorPID();
	warriorPID rightDisPID = new warriorPID();
	warriorPID leftRotatePID = new warriorPID();
	warriorPID rightRotatePID = new warriorPID();
	warriorPID disPID = new warriorPID();
	warriorPID rotatePID = new warriorPID();
	rampRate leftVelRamp;
	rampRate rightVelRamp;
	dummyPIDOutput leftdummy = new dummyPIDOutput();
	dummyPIDOutput rightdummy = new dummyPIDOutput();
	dummyPIDOutput disdummy = new dummyPIDOutput();
	dummyPIDOutput rotatedummy = new dummyPIDOutput();
	
	GripPipeline vision = new GripPipeline();
	
	public enum DriveControlState {
		 OPEN_LOOP, // open loop voltage control
	     VELOCITY_SETPOINT, // velocity PID control
	     PATH_FOLLOWING, // used for autonomous driving
	}
	
	protected static boolean usesTalonVelocityControl(DriveControlState state) {
		if (state == DriveControlState.VELOCITY_SETPOINT || state == DriveControlState.PATH_FOLLOWING) {
            return true;
        }
        return false;
	}
	
	private DriveControlState mDriveControlState;
	CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
	private boolean mIsBrakeMode;
	
	public Drivetrain() {
		mLeftMaster = CANTalonFactory.createDefaultTalon(2);
		mLeftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mLeftMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        mLeftMaster.reverseSensor(true);
        mLeftMaster.reverseOutput(false);
        CANTalon.FeedbackDeviceStatus leftSensorPresent = mLeftMaster
                .isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (leftSensorPresent != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect left encoder: " + leftSensorPresent, false);
        }
        mLeftSlave = CANTalonFactory.createPermanentSlaveTalon(3, 2);
        mLeftSlave.reverseOutput(false);
        mLeftMaster.setStatusFrameRateMs(StatusFrameRate.Feedback, 5);
        
        mRightMaster = CANTalonFactory.createDefaultTalon(0);
		mRightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mRightMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        mRightMaster.reverseSensor(false);
        mRightMaster.reverseOutput(true);
        CANTalon.FeedbackDeviceStatus rightSensorPresent = mLeftMaster
                .isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (rightSensorPresent != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect right encoder: " + rightSensorPresent, false);
        }
        mRightSlave = CANTalonFactory.createPermanentSlaveTalon(1, 0);
        mRightSlave.reverseOutput(false);
        mRightMaster.setStatusFrameRateMs(StatusFrameRate.Feedback, 5);
        
        mLeftMaster.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_10Ms);
        mLeftMaster.SetVelocityMeasurementWindow(32);
        mRightMaster.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_10Ms);
        mRightMaster.SetVelocityMeasurementWindow(32);
        
		leftVelRamp = new rampRate(1, 10);
		rightVelRamp = new rampRate(1, 10);
		leftDisPID.setPID(0.025, 0.0025, 0.05);//0.025, 0.00006
		leftDisPID.setSources(leftEncoder);
		leftDisPID.setOutputs(drivetrainLeft);
		rightDisPID.setPID(0.025, 0.0025, 0.05);
		rightDisPID.setSources(rightEncoder);
		rightDisPID.setOutputs(drivetrainRight);
		
		leftRotatePID.setPID(0.02, 0.005, 0.3);
		leftRotatePID.setSources(gyro);
		leftRotatePID.setOutputs(drivetrainLeft);
		leftRotatePID.setContinuous(false);
		rightRotatePID.setPID(0.02, 0.005, 0.3);
		rightRotatePID.setSources(gyro);
		rightRotatePID.setOutputs(drivetrainRight);
		rightRotatePID.setContinuous(false);
		
		disPID.setPID(2, 0.0, 0.0);
		disPID.setSources(RobotMap.averageEncoderDistance);
		disPID.setOutputs(disdummy);

		rotatePID.setPID(1, 0, 0.02);
		rotatePID.setSources(gyro);
		rotatePID.setOutputs(rotatedummy);
		rotatePID.setOutputRange(-10, 10);
		rotatePID.setInputRange(-180, 180);
		rotatePID.setContinuous(true);
	}
	
	public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mLeftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            mLeftMaster.configNominalOutputVoltage(0.0, 0.0);
            mRightMaster.configNominalOutputVoltage(0.0, 0.0);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            setBrakeMode(false);
        }
        // Right side is reversed, but reverseOutput doesn't invert PercentVBus.
        // So set negative on the right master.
        mRightMaster.set(-signal.getRight());
        mLeftMaster.set(signal.getLeft());
    }
	
	public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            mRightMaster.enableBrakeMode(on);
            mRightSlave.enableBrakeMode(on);
            mLeftMaster.enableBrakeMode(on);
            mLeftSlave.enableBrakeMode(on);
        }
    }
	
	//@Override
	public void outputToSmartDashboard() {
		
	}
	
    public synchronized void freeze() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }
    
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        setVelocity(left_inches_per_sec, right_inches_per_sec);
    }
    
    private void configureTalonsForSpeedControl() {
        if (!usesTalonVelocityControl(mDriveControlState)) {
            // We entered a velocity control state.
            mLeftMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            mLeftMaster.setNominalClosedLoopVoltage(12.0);
            mLeftMaster.setProfile(kHighGearVelocityControlSlot);
            mLeftMaster.configNominalOutputVoltage(0.5, -0.5);
            mLeftMaster.setPID(0.22, 0.0, 0.0);
            mLeftMaster.setF(0.1097);
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            mRightMaster.setNominalClosedLoopVoltage(12.0);
            mRightMaster.setProfile(kHighGearVelocityControlSlot);
            mRightMaster.configNominalOutputVoltage(0.5, -0.5);
            mRightMaster.setPID(0.22, 0.0, 0.0);
            mRightMaster.setF(0.1097);
            setBrakeMode(true);
        }
    }
	
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            configureTalonsForSpeedControl();
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed, new PathFollower.Parameters(new Lookahead(12.0, 24.0, 9.0, 80), 0.0, 5.0, 0.03, 0.02, 1.0, 0.05, 80, 120, 0.75, 12.0, 9.0));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setVelocitySetpoint(0, 0);
        }
    }
    
    /**
     * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
     * pose, distance driven, and velocity, the updates the wheel velocity setpoints.
     */
    public void updatePathFollower(double timestamp, double sign) {
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose, RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
        if (!mPathFollower.isFinished()) {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            setVelocity(sign*setpoint.left, sign*setpoint.right);
        } else {
            setVelocity(0, 0);
        }
    }
    
    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }
  
    public synchronized void setGyroAngle() {
        gyro.reset();
    }
    
	Timer t = new Timer();
	private final double critical = 3.15 * 3.141 / 256;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
	//	setDefaultCommand(new drive());
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
	    			
	    			if (r.width/r.height < 5 && r.width/r.height > 3) {
	    				SmartDashboard.putNumber("width" + i, r.width);
		    			SmartDashboard.putNumber("height" + i, r.height);
		    			centerX = r.x + r.width / 2;
		    			centerY = r.y + r.height / 2;
		    			center.add(new Point(centerX, centerY));
	    			}
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
	    		//angle += 1;
	    		SmartDashboard.putNumber("angle", angle);
	    		return angle;
	    	}
    }
	
	public void curveDrive(double distance, double angle) {
		leftDisPID.setOutputs(leftdummy); //don't use this!!!
		leftDisPID.setOutputRange(-1, 1);
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
	
	public boolean curve1Drive(double distance, double r, double tolerance) {
		rightDisPID.setOutputs(rightdummy);
		rightDisPID.setOutputRange(-1, 1);
		rightDisPID.setSetpoint(-distance);
		rightDisPID.setAbsoluteTolerance(tolerance);
		rightDisPID.enable();
		double curve = Math.pow(Math.E, -r/RobotMap.ROBOT_WIDTH);
		if (rightDisPID.isOnTarget())
			return true;
		else {
			myDrive.drive(-rightdummy.getOutput(), curve);
			return false;
		}
	}
	
	public boolean curveVelDrive(double setpoint, double abs_dis_tolerance) {
		return (leftEncoder.getDistance() - rightEncoder.getDistance()) / 2 > setpoint - abs_dis_tolerance;
	}
	
	public void setVelocity(double left, double right) {
		if (usesTalonVelocityControl(mDriveControlState)) {
            final double max_desired = Math.max(Math.abs(left), Math.abs(right));
            final double scale = max_desired > 80 ? 80 / max_desired : 1.0;
            SmartDashboard.putNumber("scale", scale);
            mLeftMaster.set(inchesPerSecondToRpm(-left * scale));
            mRightMaster.set(inchesPerSecondToRpm(-right * scale));
        } else {
            System.out.println("Hit a bad velocity control state");
            mLeftMaster.set(0);
            mRightMaster.set(0);
        }
	}
	
	public synchronized void splineDrive(double distance, double maxSpeed, double angle, double curvature, double tolerance) {
		configureTalonsForSpeedControl();
        mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        disPID.setSetpoint(distance);
		disPID.setOutputRange(-maxSpeed, maxSpeed);
        disPID.enable();
        
	}
	
	public synchronized void updateSplineDrive(double angle) {
		double strikeVelocity = disdummy.getOutput();
		double leftVelocity = strikeVelocity + angle;
		double rightVelocity = strikeVelocity - angle;
		setVelocity(leftVelocity, rightVelocity);
	}
	
	private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }
	
	private static double inchesToRotations(double inches) {
        return inches / (3.1 * Math.PI);
    }
	
	public boolean driveTo(double distance, double tolerance) {
		leftDisPID.setSetpoint(distance);
		rightDisPID.setOutputs(drivetrainRight);
		leftDisPID.setOutputs(drivetrainLeft);
		leftDisPID.setOutputRange(-1, 1);
	    	rightDisPID.setSetpoint(-distance);
	    	leftDisPID.setAbsoluteTolerance(tolerance);
	    	rightDisPID.setAbsoluteTolerance(tolerance);
	    	leftDisPID.enable();
	    	rightDisPID.enable();
		
	    	return (leftDisPID.isOnTarget() && rightDisPID.isOnTarget());
	}
	public boolean turnTo(double angle, double tolerance) {
		leftRotatePID.setSetpoint(angle);
		rightDisPID.setSetpoint(0);
		//rightRotatePID.setSetpoint(angle);
		leftRotatePID.setAbsoluteTolerance(tolerance);
		//rightRotatePID.setAbsoluteTolerance(tolerance);
		leftRotatePID.enable();
		rightDisPID.enable();
		//rightRotatePID.enable();
		
		return leftRotatePID.isOnTarget();
	}
	public boolean driveForwardDistance(double distance) {
		boolean i = true;
		while (i) 
		{		
			double leftEncoderCount = leftEncoder.get();
			double rightEncoderCount = rightEncoder.get();
			double leftEncoderDistance = leftEncoderCount * critical; // Distance in Inches	
			double rightEncoderDistance = rightEncoderCount * critical;
			
			if (leftEncoderDistance < distance && rightEncoderDistance < distance && (gyro.getAngle() <= 1.5 && gyro.getAngle() >= -1.5))
			{
				myDrive.drive(1, 0);
			}
			else if (leftEncoderDistance < distance && rightEncoderDistance < distance && gyro.getAngle() > 1) {
				myDrive.drive(1, -0.1);
			}
			else if (leftEncoderDistance < distance && rightEncoderDistance < distance && gyro.getAngle() < -1) {
				myDrive.drive(1, 0.1);
			}
			//else if (t.get() > 6) {
				//i = false;
			//}
			else {
				i = false;
			}		
		}
		//myDrive.drive(0, 0);
		return true;
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
			double rightEncoderDistance = -rightEncoderCount * critical;

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
		gyro.reset();
		leftRotatePID.setSetpoint(degrees);
		rightRotatePID.setSetpoint(degrees);
		leftRotatePID.enable();
		rightRotatePID.enable();
	}
	
	public void shiftHigh() {
		cylinder.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void shiftLow() {
		cylinder.set(DoubleSolenoid.Value.kForward);
	}
	
	public void warriorDrive(double y, double z) {
		double crap;
		if (z < 0)
			crap = Math.pow(z, 2);
		else 
			crap = -Math.pow(z, 2);
		myDrive.arcadeDrive(-y, crap);
	}
	
	public void zeroSensors() {
		leftEncoder.reset();
		rightEncoder.reset();
		gyro.reset();
	}
	
	public void stop() {
		leftDisPID.disable();
		rightDisPID.disable();
		leftRotatePID.disable();
		rightRotatePID.disable();
		rotatePID.disable();
		setOpenLoop(DriveSignal.NEUTRAL);
		//myDrive.drive(0,0);
	}
}
