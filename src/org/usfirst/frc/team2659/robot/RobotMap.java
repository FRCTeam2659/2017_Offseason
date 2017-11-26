
package org.usfirst.frc.team2659.robot;

import org.usfirst.frc.team2659.robot.util.SCWrapper;
import org.usfirst.frc.team2659.robot.util.encoderWrapper;

import com.ctre.CANTalon;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistributionPanel;


public class RobotMap {
	
	public static RobotDrive myRobot;
	
	public static CANTalon leftFrontSC;
	public static CANTalon leftRearSC;
	public static CANTalon rightFrontSC;
	public static CANTalon rightRearSC;
	public static PWMSpeedController climberSC;
	public static PWMSpeedController intakeSC;
	public static PowerDistributionPanel pdp;
	public static PIDController leftFrontPID;
	public static PIDController leftRearPID;
	
	public static SCWrapper drivetrainLeft, drivetrainRight;
	
    public static DoubleSolenoid intakeCylinder;
    public static DoubleSolenoid shiftCylinder;
    
    public static AnalogInput gearSensor;
	public static Encoder leftEncoder;
    public static Encoder rightEncoder;
    public static ADXRS450_Gyro gyro;
    
    public static encoderWrapper leftRateEncoder;
    public static encoderWrapper rightRateEncoder;
    
    public static UsbCamera boilerCamera;
    
    public final static int ROBOT_WIDTH = 25;
    
    public static void init() {
    	pdp = new PowerDistributionPanel();
    	
    	leftFrontSC = new CANTalon(2);
    	leftFrontSC.setInverted(true);
    	
    	leftRearSC = new CANTalon(3);
    	leftRearSC.setInverted(true);
    	
    	rightFrontSC = new CANTalon(0);
    	rightFrontSC.setInverted(true);
    	
    	rightRearSC = new CANTalon(1);
    	rightRearSC.setInverted(true);
    	
    	drivetrainLeft = new SCWrapper(leftFrontSC, leftRearSC);
    	drivetrainRight = new SCWrapper(rightFrontSC, rightRearSC);
    	
    	myRobot = new RobotDrive(drivetrainLeft, drivetrainRight);
    	
    	climberSC = new VictorSP(4);
    	   	
    	intakeSC = new VictorSP(5);
    	
    	intakeCylinder = new DoubleSolenoid(0, 1);
   	
    	shiftCylinder = new DoubleSolenoid(2, 3);
	
    	gearSensor = new AnalogInput(3);
    	LiveWindow.addSensor("Intake", "Proxy Sensor", (AnalogInput) gearSensor);
    	
    	leftEncoder = new Encoder(0,1,false);
    	leftEncoder.setDistancePerPulse(3.15*Math.PI/256);
    	leftEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
    	LiveWindow.addSensor("Drivetrain", "Left Encoder", (Encoder) leftEncoder);
    	
    	rightEncoder = new Encoder(2,3,false);
    	rightEncoder.setDistancePerPulse(3.15*Math.PI/256);
    	rightEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
    	LiveWindow.addSensor("Drivetrain", "Right Encoder", (Encoder) rightEncoder);
    	
    	leftRateEncoder = new encoderWrapper(leftEncoder, PIDSourceType.kRate);
    	rightRateEncoder = new encoderWrapper(rightEncoder, PIDSourceType.kRate);
    	
    	gyro = new ADXRS450_Gyro();
    	gyro.setPIDSourceType(PIDSourceType.kDisplacement);
    	gyro.calibrate();
    	gyro.reset();
    	
    boilerCamera = CameraServer.getInstance().startAutomaticCapture(0);
    boilerCamera.setExposureAuto();
    	boilerCamera.setResolution(640, 480);
    }
    public static void periodic() {
    	SmartDashboard.putNumber("Left Encoder", leftEncoder.getDistance());
    	SmartDashboard.putNumber("Right Encoder", -rightEncoder.getDistance());
    	SmartDashboard.putNumber("gyro", gyro.getAngle());
    	//SmartDashboard.putNumber("pdp 0", pdp.getCurrent(0));
    	//SmartDashboard.putNumber("pdp 1", pdp.getCurrent(1));
    	//SmartDashboard.putNumber("pdp 14", pdp.getCurrent(14));
    	//SmartDashboard.putNumber("pdp 15", pdp.getCurrent(15));
    	SmartDashboard.putNumber("Gear Sensor", gearSensor.getVoltage());
    }
}
