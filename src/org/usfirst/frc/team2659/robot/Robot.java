package org.usfirst.frc.team2659.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2659.robot.commands.commandGroup.*;
import org.usfirst.frc.team2659.robot.subsystems.*;
import org.usfirst.frc.team2659.robot.util.CsvLogger;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static OI oi;
	public static Drivetrain drivetrain;
    public static Climber climber;
    public static GearIntake intake;
    public static PowerDistributionPanel pdp = new PowerDistributionPanel();
	private static SendableChooser<CommandGroup> autoChooser = new SendableChooser<CommandGroup>();
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	@Override
	public void robotInit() {
		RobotMap.init();
		
		drivetrain = new Drivetrain();
		climber = new Climber();
		intake = new GearIntake();
		
		oi = new OI();	
		RobotMap.gyro.calibrate();
		//autoChooser.addObject("Auto drive() method", new AutoRight1());
		autoChooser.addDefault("Auto Blue Gear Hopper", new BlueGearHopper());
		SmartDashboard.putData("AUTO", autoChooser);
		
		//initLoggingChannels();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		drivetrain.stop();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		//autonomousCommand = new AutoStraight();
		drivetrain.zeroSensors();

		//autonomousCommand = (Command) autoChooser.getSelected();
		//autonomousCommand.start();
		//CsvLogger.init();
		Scheduler.getInstance().enable();
		Scheduler.getInstance().add((CommandGroup) autoChooser.getSelected());
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		//CsvLogger.logData(false);
	}

	@Override
	public void teleopInit() {
		Scheduler.getInstance().enable();
		CsvLogger.init();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		RobotMap.periodic();
		//CsvLogger.logData(false);
	}
	
	public void initLoggingChannels() {
		CsvLogger.addLoggingFieldDouble("TIME", "sec", "getFPGATimestamp", Timer.class);
		CsvLogger.addLoggingFieldDouble("left_velocity", "in/s", "getRate", RobotMap.leftEncoder);
		CsvLogger.addLoggingFieldDouble("right_velocity", "in/s", "getRate", RobotMap.rightEncoder);
		CsvLogger.preCacheAllMethods();
	}
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
