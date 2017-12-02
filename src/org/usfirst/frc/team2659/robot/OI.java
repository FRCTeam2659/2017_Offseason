package org.usfirst.frc.team2659.robot;


import org.usfirst.frc.team2659.robot.commands.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public Joystick driveStick;
	public Joystick operatorStick;
	
	public Button driveButton1;
	public Button driveButton2;
	public Button driveButton3;
	public Button driveButton10;
	
	public Button operatorButton1;
	public Button operatorButton2;
	public Button operatorButton3;
	public Button operatorButton4;
	public Button operatorButton5;
	public Button operatorButton6;
	public Button operatorButton7;
	public Button operatorButton8;
	
	public OI() {	
		driveStick = new Joystick(0);
		operatorStick = new Joystick(1);
		
		operatorButton3 = new JoystickButton(driveStick, 2);
		operatorButton3.whileHeld(new GearIntakeOut());
		
		operatorButton1 = new JoystickButton(operatorStick, 3);
		operatorButton1.whileHeld(new GearIntakeIn());
		
		operatorButton2 = new JoystickButton(operatorStick, 1);
		operatorButton2.whileHeld(new GearIntakeDown());
		
		operatorButton4 = new JoystickButton(operatorStick, 4);
		operatorButton4.whileHeld(new GearIntakeUp());
		
		operatorButton5 = new JoystickButton(operatorStick, 6);
		operatorButton5.whileHeld(new GearOutakeAuto());
		operatorButton5.whenReleased(new stopIntake());
		
		operatorButton7 = new JoystickButton(driveStick, 1);//5
		operatorButton7.whileHeld(new GearIntakeAuto());
		operatorButton7.whenReleased(new GearIntakeUp());
		
		operatorButton8 = new JoystickButton(driveStick, 4);//6
		operatorButton8.whileHeld(new ScoreGear());
		operatorButton8.whenReleased(new stopIntake());
		
		operatorButton5 = new JoystickButton(operatorStick, 7);
		operatorButton5.whileHeld(new climb());
		
		driveButton2 = new JoystickButton(driveStick, 7);
		driveButton2.whileHeld(new shiftLow());
		
		driveButton1 = new JoystickButton(driveStick, 8);
		driveButton1.whileHeld(new shiftHigh());	
		
		driveButton3 = new JoystickButton(driveStick, 3);
		driveButton3.whenPressed(new aim());
		
		driveButton10 = new JoystickButton(driveStick, 10);
		driveButton10.whenPressed(new AutoStraight());
	}
	 
	public Joystick getjoystick() {
		return driveStick;
	}
}
