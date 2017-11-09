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
		
		operatorButton1 = new JoystickButton(operatorStick, 1);
		operatorButton1.whileHeld(new GearIntakeIn());
		
		operatorButton3 = new JoystickButton(operatorStick, 3);
		operatorButton3.whileHeld(new GearIntakeOut());
		
		operatorButton2 = new JoystickButton(operatorStick, 2);
		operatorButton2.whileHeld(new GearIntakeDown());
		
		operatorButton4 = new JoystickButton(operatorStick, 4);
		operatorButton4.whileHeld(new GearIntakeUp());
		
		operatorButton5 = new JoystickButton(operatorStick, 5);
		operatorButton5.whileHeld(new climb());
		
		/*operatorButton5 = new JoystickButton(operatorStick, 6);
		operatorButton5.whileHeld(new GearOutakeAuto());
		operatorButton5.whenReleased(new stopIntake());*/
		
		operatorButton7 = new JoystickButton(operatorStick, 7);
		operatorButton7.whileHeld(new GearIntakeAuto());
		operatorButton7.whenReleased(new GearIntakeUp());
		
		operatorButton8 = new JoystickButton(operatorStick, 8);
		operatorButton8.whileHeld(new ScoreGear());
		operatorButton8.whenReleased(new stopIntake());
		
		driveButton2 = new JoystickButton(driveStick, 7);
		driveButton2.whileHeld(new shiftLow());
		
		driveButton1 = new JoystickButton(driveStick, 8);
		driveButton1.whileHeld(new shiftHigh());	
	}
	 
	public Joystick getjoystick() {
		return driveStick;
	}
}
