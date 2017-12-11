package org.usfirst.frc.team2659.robot;


import org.usfirst.frc.team2659.robot.commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {
	
	public XboxController driveStick;
	public Joystick operatorStick;
	
	private Button driveButton1;
	private Button driveButton2;
	private Button driveButton3;
	private Button driveButton10;
	public Button driveReverseButton;
	public Button driveNormalButton;
	
	private Button operatorButton1;
	private Button operatorButton2;
	private Button operatorButton3;
	private Button operatorButton4;
	private Button operatorButton5;
	private Button operatorButton7;
	private Button operatorButton8;
	
	public OI() {	
		driveStick = new XboxController(0);
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
		
		driveReverseButton = new JoystickButton(driveStick, 6);
		
		driveNormalButton = new JoystickButton(driveStick, 5);
	}
	 
	public XboxController getjoystick() {
		return driveStick;
	}
}
