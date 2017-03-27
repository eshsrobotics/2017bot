package org.usfirst.frc.team1759.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;

import org.usfirst.frc.team1759.robot.commands.AutoAimBoilerCommand;
import org.usfirst.frc.team1759.robot.subsystems.MecanumDriveSubSystem;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
@SuppressWarnings("unused")
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());

	public Joystick leftStick;
	public Joystick rightStick;

	/**
	 * This is an alias for the left joystick (for now.)
	 */
	public Joystick shootStick;

	public double thresholdedX;
	public double thresholdedY;
	public double thresholdedTwist;

	public final double thresholdX = 0; // Added to make sure the drive isn't
										// too sensitive
	public final double thresholdY = 0; // As above
	public final double thresholdTwist = 0; // As above

	/**
	 * The button we use to enter auto-aiming mode when held down.
	 * 
	 * Ultimately, this button will be on the shootingStick, but we have it on
	 * the leftStick for the time being.
	 */
	public static final int AUTO_AIM_BUTTON_NUMBER = 2;
	public static final int GYRO_SWITCH_BUTTON_NUMBER = 11;
	public static final int INCREASE_SPEED = 12;
	public static final int DECREASE_SPEED = 11;

	public static final int GEAR_IN = 10;
	public static final int GEAR_OUT = 9;
	public static final int BALL_IN = 8;
	public static final int BALL_OUT = 7;

	Button autoAimButton = new JoystickButton(leftStick, AUTO_AIM_BUTTON_NUMBER);
	Button driveSwitch = new JoystickButton(rightStick, GYRO_SWITCH_BUTTON_NUMBER);
	Button goFast = new JoystickButton(leftStick, INCREASE_SPEED);
	Button goSlow = new JoystickButton(leftStick, DECREASE_SPEED);
	Button gearIn = new JoystickButton(leftStick, GEAR_IN);
	Button gearOut = new JoystickButton(leftStick, GEAR_OUT);
	Button ballIn = new JoystickButton(leftStick, BALL_IN);
	Button ballOut = new JoystickButton(leftStick, BALL_OUT);

	/**
	 * Constructs this input interface and provides it with everything it needs
	 * to control the subsystems it uses.
	 * 
	 * @param mecanumDriveSubSystem
	 *            The drive used for the {@link AutoAimBoilerCommand}.
	 * @param serverRunnable
	 *            The {@link PapasData} source used for the
	 *            {@link AutoAimBoilerCommand}.
	 */
	public OI(MecanumDriveSubSystem mecanumDriveSubSystem, ServerRunnable serverRunnable) {
		this.leftStick = new Joystick(0);
		this.rightStick = new Joystick(1);
		this.shootStick = this.leftStick;

		Button autoAimButton = new JoystickButton(rightStick, AUTO_AIM_BUTTON_NUMBER);

		///////////////////////
		// Program the buttons.

		autoAimButton.whileHeld(new AutoAimBoilerCommand(mecanumDriveSubSystem, serverRunnable));
	}

	/**
	 * The function is to let us make the joystick movements be less likely to
	 * create a sudden change on the field. It helps us from causing massive
	 * changes.
	 *
	 **/

	public void limitThreshold() {
		thresholdedX = this.rightStick.getX();
		thresholdedY = this.rightStick.getY();
		thresholdedTwist = this.rightStick.getTwist();

		// clamp values that are too low.
		if (Math.abs(thresholdedX) < thresholdX) {
			this.thresholdedX = 0;
		}
		if (Math.abs(this.thresholdedY) < thresholdY) {
			this.thresholdedY = 0;
		}
		if (Math.abs(this.thresholdedTwist) < thresholdTwist) {
			this.thresholdedTwist = 0;
		}
	}

}
