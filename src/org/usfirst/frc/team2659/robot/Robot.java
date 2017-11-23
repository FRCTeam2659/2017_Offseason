
package org.usfirst.frc.team2659.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2659.robot.commands.*;
import org.usfirst.frc.team2659.robot.subsystems.*;


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
    
   /* private static final int IMG_WIDTH = 640;
	private static final int IMG_HEIGHT = 480;
	
	private VisionThread visionThread;
	private double centerX = 0.0;
	
	
	private final Object imgLock = new Object();*/

    Command autonomousCommand;
	SendableChooser<Command> autoChooser;

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
		
		autoChooser = new SendableChooser<Command>();
		autoChooser.addDefault("Auto 1 - Straight Gear", new AutoStraight());
		autoChooser.addObject("Auto 2 - Right Gear", new AutoRight());
		autoChooser.addObject("Auto 3 - Left Gear", new AutoLeft());
		SmartDashboard.putData("Autonomous chooser", autoChooser);
		
		/*new Thread(() -> {
	        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("aim", 0);
	            camera.setResolution(640, 480);
			}).start();
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
	    
	    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
	        if (!pipeline.filterContoursOutput().isEmpty()) {
	            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	       
	            ArrayList<MatOfPoint> result = pipeline.filterContoursOutput();
	            synchronized (imgLock) {
	                ArrayList<Point> center = new ArrayList<Point>();
		        		for (int i = 0; i < result.size(); i++) {
		        			r = Imgproc.boundingRect(result.get(i));
		        			centerX = r.x + r.width / 2;
		        			center.add(new Point(centerX, 0));
		        		}
		        		centerX = 0;
		        		
		        		for (int i = 0; i < center.size(); i++) {
		            		centerX += center.get(i).x;
		            		
		            	}
		            	centerX /= center.size();
		    
	            }
	            
	        }
	    });
	    visionThread.start();*/
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

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
		autonomousCommand = (Command) autoChooser.getSelected();
		autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		RobotMap.periodic();
		/*double centerX;
	    synchronized (imgLock) {
	        centerX = this.centerX;
	    }
	    double turn = centerX - (IMG_WIDTH / 2);
	    SmartDashboard.putNumber("rotate value", turn);*/
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
