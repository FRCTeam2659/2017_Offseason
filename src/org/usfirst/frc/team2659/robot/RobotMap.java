
package org.usfirst.frc.team2659.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
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
	
    public static DoubleSolenoid intakeCylinder;
    public static DoubleSolenoid shiftCylinder;
    
    public static AnalogInput gearSensor;
	public static Encoder leftEncoder;
    public static Encoder rightEncoder;
    public static ADXRS450_Gyro gyro;
    
    
    
    public static void init() {
    	pdp = new PowerDistributionPanel();
    	
    	leftFrontSC = new CANTalon(2);
    	leftFrontSC.setInverted(true);
    	LiveWindow.addActuator("Drivetrain", "Left Front", (CANTalon) leftFrontSC);
    	
    	leftRearSC = new CANTalon(3);
    	LiveWindow.addActuator("Drivetrain", "Left Rear", (CANTalon) leftRearSC);
    	leftRearSC.setInverted(true);
    	
    	rightFrontSC = new CANTalon(0);
    	rightFrontSC.setInverted(true);
    	LiveWindow.addActuator("Drivetrain", "Right Front", (CANTalon) rightFrontSC);
    	
    	rightRearSC = new CANTalon(1);
    	rightRearSC.setInverted(true);
    	LiveWindow.addActuator("Drivetrain", "Right Rear", (CANTalon) rightRearSC);
    	
    	myRobot = new RobotDrive(leftFrontSC, leftRearSC, rightFrontSC, rightRearSC);
    	
    	climberSC = new VictorSP(4);
    	LiveWindow.addActuator("Climber", "Motors", (VictorSP) climberSC);
    	   	
    	intakeSC = new VictorSP(5);
    	LiveWindow.addActuator("Intake", "Main", (VictorSP) intakeSC);
    	
    	intakeCylinder = new DoubleSolenoid(0, 1);
    	LiveWindow.addActuator("Intake", "Actuate Cylinders", (DoubleSolenoid) intakeCylinder);
    	
    	shiftCylinder = new DoubleSolenoid(2, 3);
    	LiveWindow.addActuator("Drivetrain", "Shift Cylinders", (DoubleSolenoid) shiftCylinder);
    	
    	gearSensor = new AnalogInput(3);
    	LiveWindow.addSensor("Intake", "Proxy Sensor", (AnalogInput) gearSensor);
    	
    	leftEncoder = new Encoder(0,1,false);
    	LiveWindow.addSensor("Drivetrain", "Left Encoder", (Encoder) leftEncoder);
    	
    	rightEncoder = new Encoder(2,3,false);
    	LiveWindow.addSensor("Drivetrain", "Right Encoder", (Encoder) rightEncoder);
    	
    	gyro = new ADXRS450_Gyro();
    	gyro.setPIDSourceType(PIDSourceType.kDisplacement);
    	gyro.calibrate();
    	gyro.reset();
    }
    public static void periodic() {
    	SmartDashboard.putNumber("Left Encoder", leftEncoder.get());
    	SmartDashboard.putNumber("Right Encoder", -rightEncoder.get());
    	SmartDashboard.putNumber("gyro", gyro.getAngle());
    	//SmartDashboard.putNumber("pdp 0", pdp.getCurrent(0));
    	//SmartDashboard.putNumber("pdp 1", pdp.getCurrent(1));
    	//SmartDashboard.putNumber("pdp 14", pdp.getCurrent(14));
    	//SmartDashboard.putNumber("pdp 15", pdp.getCurrent(15));
    	SmartDashboard.putNumber("Gear Sensor", gearSensor.getVoltage());
    }
}
