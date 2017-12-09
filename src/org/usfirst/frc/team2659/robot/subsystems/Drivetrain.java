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
import org.usfirst.frc.team2659.robot.commands.drive;
import org.usfirst.frc.team2659.robot.util.drivers.CANTalonFactory;
import org.usfirst.frc.team2659.robot.util.DriveSignal;
import org.usfirst.frc.team2659.robot.util.GripPipeline;
import org.usfirst.frc.team2659.robot.util.control.Lookahead;
import org.usfirst.frc.team2659.robot.util.control.Path;
import org.usfirst.frc.team2659.robot.util.control.PathFollower;
import org.usfirst.frc.team2659.robot.util.math.RigidTransform2d;
import org.usfirst.frc.team2659.robot.util.SCWrapper;
import org.usfirst.frc.team2659.robot.util.math.Twist2d;
import org.usfirst.frc.team2659.robot.util.dummyPIDOutput;
import org.usfirst.frc.team2659.robot.util.encoderWrapper;
import org.usfirst.frc.team2659.robot.util.control.warriorPID;

import com.ctre.CANTalon;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.VelocityMeasurementPeriod;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drivetrain extends Subsystem {

    private Path mCurrentPath = null;
    private RobotState mRobotState = RobotState.getInstance();
    private PathFollower mPathFollower;
    private boolean isHighGear = false;
	Timer t = new Timer();
	private final double critical = 3.15 * 3.141 / 256;
	
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
	dummyPIDOutput leftdummy = new dummyPIDOutput();
	dummyPIDOutput rightdummy = new dummyPIDOutput();
	dummyPIDOutput disdummy = new dummyPIDOutput();
	dummyPIDOutput rotatedummy = new dummyPIDOutput();
	
	GripPipeline vision = new GripPipeline();
	
	public enum DriveControlState {
		 OPEN_LOOP, // open loop voltage control
	     VELOCITY_SETPOINT, // velocity PID control
	     PATH_FOLLOWING, // used for autonomous driving
	     TURN_TO_HEADING, // turn in place
	}
	
	protected static boolean usesTalonVelocityControl(DriveControlState state) {
		if (state == DriveControlState.VELOCITY_SETPOINT || state == DriveControlState.PATH_FOLLOWING) {
            return true;
        }
        return false;
	}
	
	protected static boolean usesTalonPositionControl(DriveControlState state) {
        if (state == DriveControlState.TURN_TO_HEADING)
            return true; 
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
        
		leftDisPID.setPID(0.18, 0.0025, 0.056);//0.025, 0.00006
		leftDisPID.setSources(leftEncoder);
		leftDisPID.setOutputs(drivetrainLeft);
		rightDisPID.setPID(0.18, 0.0025, 0.056);
		rightDisPID.setSources(rightEncoder);
		rightDisPID.setOutputs(drivetrainRight);
		
		leftRotatePID.setPID(0.043, 0.0062, 0.3);
		leftRotatePID.setSources(gyro);
		leftRotatePID.setOutputs(drivetrainLeft);
		leftRotatePID.setContinuous(false);
		rightRotatePID.setPID(0.043, 0.0062, 0.3);
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
		
		reloadGains();
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
        		shiftLow();
            mLeftMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            mLeftMaster.setNominalClosedLoopVoltage(12.0);
            mLeftMaster.setProfile(1); //set the profile always to 1 for speed control
            mLeftMaster.configNominalOutputVoltage(0.5, -0.5);
            
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            mRightMaster.setNominalClosedLoopVoltage(12.0);
            mRightMaster.setProfile(1);
            mRightMaster.configNominalOutputVoltage(0.5, -0.5);
            
            setBrakeMode(true);
        }
    }
    
    private void configureTalonsForPositionControl() {
        if (!usesTalonPositionControl(mDriveControlState)) {
            // We entered a position control state.
        		shiftLow();
            mLeftMaster.changeControlMode(CANTalon.TalonControlMode.MotionMagic);
            mLeftMaster.setNominalClosedLoopVoltage(12.0);
            mLeftMaster.setProfile(0);
            mLeftMaster.configNominalOutputVoltage(0.5,-0.5);
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.MotionMagic);
            mRightMaster.setNominalClosedLoopVoltage(12.0);
            mRightMaster.setProfile(0);
            mRightMaster.configNominalOutputVoltage(0.5,-0.5);
            setBrakeMode(true);
        }
    }
    
    public synchronized void updatePositionSetpoint(double left_position_inches, double right_position_inches) {
        if (usesTalonPositionControl(mDriveControlState)) {
            mLeftMaster.set(left_position_inches);
            mRightMaster.set(right_position_inches);
            SmartDashboard.putString("success", "yup");
        } else {
            System.out.println("Hit a bad position control state");
            mLeftMaster.set(0);
            mRightMaster.set(0);
            SmartDashboard.putString("success", "nope");
        }
    }
    
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            configureTalonsForSpeedControl();
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed, new PathFollower.Parameters(new Lookahead(12.0, 24.0, 6.0, 75), 0.0, 5.0, 0.03, 0.02, 1.0, 0.05, 75, 120, 0.75, 12.0, 9.0));
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
    public void updatePathFollower(double timestamp) {
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose, RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
        if (!mPathFollower.isFinished()) {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            setVelocity(setpoint.left, setpoint.right);
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
	    		return 0;
	    	}
	    	else {
	    		for (int i = 0; i < result.size(); i++) {
	    			Rect r = Imgproc.boundingRect(result.get(i));
	    			
	    			//if (r.width/r.height < 5 && r.width/r.height > 3) {
		    			centerX = r.x + r.width / 2;
		    			centerY = r.y + r.height / 2;
		    			center.add(new Point(centerX, centerY));
	    			//}
	    		}
	    	}
	    	
	    	centerX = centerY = 0;
	    	for (int i = 0; i < center.size(); i++) {
	    		centerX += center.get(i).x;
	    		//centerY += center.get(i).y;
	    	}
	    	centerX /= center.size();
	    	//centerY /= center.size();
	    	
	    	double angle;
	    	if (centerX <= 319.5) {
	    		angle = Math.atan((319.5-centerX)/319.5/1.6512)/Math.PI*180; //the camera actual angle is 62.4
	    		SmartDashboard.putNumber("angle1", -angle);
	    		return -angle;
	    	}
	    	else {
	    		angle = Math.atan((centerX-319.5)/319.5/1.6512)/Math.PI*180;
	    		SmartDashboard.putNumber("angle2", angle);
	    		return angle;
	    	}
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
	
    private static double rotationsToInches(double rotations) {
        return rotations * (3.1 * Math.PI);
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
	
	public void rotateDistance(double angle) {
       /* configureTalonsForPositionControl();
        mDriveControlState = DriveControlState.TURN_TO_HEADING;
        updatePositionSetpoint(leftEncoder.getDistance() - wheel_delta.left, rightEncoder.getDistance() - wheel_delta.right);*/
		Kinematics.DriveVelocity wheel_delta = Kinematics.inverseKinematics(new Twist2d(0,0,angle/180*Math.PI));
		SmartDashboard.putNumber("leftdealta", wheel_delta.left);
        leftDisPID.setSetpoint(leftEncoder.getDistance()-wheel_delta.left);
        rightDisPID.setSetpoint(rightEncoder.getDistance()+wheel_delta.right);
        leftDisPID.enable();
        rightDisPID.enable();
	}
	
	public void shiftHigh() {
		cylinder.set(DoubleSolenoid.Value.kReverse);
		isHighGear = true;
	}
	
	public void shiftLow() {
		cylinder.set(DoubleSolenoid.Value.kForward);
		isHighGear = false;
	}
	
	public boolean isHighGear() {
		return isHighGear;
	}
	public void warriorDrive(double y, double z) {
		double crap;
		if (z < 0)
			crap = Math.pow(z, 2);
		else 
			crap = -Math.pow(z, 2);
		myDrive.arcadeDrive(-y, crap);
	}
	
	public void reloadGains() {
		mLeftMaster.setPID(1.0, 0.002, 100.0, .45, 700, 240, 0);
		mLeftMaster.setMotionMagicCruiseVelocity(75);
		mLeftMaster.setMotionMagicAcceleration(200);
		mRightMaster.setPID(1.0, 0.002, 100.0, .45, 700, 240, 0);
		mRightMaster.setMotionMagicCruiseVelocity(75);
		mRightMaster.setMotionMagicAcceleration(200);
		mLeftMaster.setVoltageCompensationRampRate(0.0);
		mRightMaster.setVoltageCompensationRampRate(0.0);
		
		mLeftMaster.setPID(0.86, 0.0, 6.0, .15, 0, 240, 1);
		mRightMaster.setPID(0.86, 0.0, 6.0, .15, 0, 240, 1);
	}
	
	public void zeroSensors() {
		leftEncoder.reset();
		rightEncoder.reset();
		mLeftMaster.setEncPosition(0);
		mLeftMaster.setPosition(0);
		mRightMaster.setEncPosition(0);
		mRightMaster.setPosition(0);
		mLeftSlave.setPosition(0);
		mRightSlave.setPosition(0);
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
