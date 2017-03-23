package org.usfirst.frc.team1759.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Ari Berkowicz
 * 
 *         The VM is configured to automatically run this class, and to call the
 *         functions corresponding to each mode, as described in the
 *         IterativeRobot documentation. If you change the name of this class or
 *         the package after creating this project, you must also update the
 *         manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	public static final double thresholdX = .35;
	public static final double thresholdY = .2;
	public static final double thresholdTwist = .2;

	double speed;

	// public static final ExampleSubsystem exampleSubsystem = new
	// ExampleSubsystem();

	Command autonomousCommand;
	SendableChooser chooser;
	String autoSelected;

	RobotDrive myRobot;
	Joystick rightStick;
	Jaguar back_right_wheel;
	Jaguar front_right_wheel;
	Jaguar back_left_wheel;
	Jaguar front_left_wheel;
	Jaguar feeder;
	Jaguar shooter;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {

		// Define Autonomous mode options and display on Driver station dash.
		chooser = new SendableChooser();
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		/*
		 * If you draw an imaginary "И" (Cyrillic ee) on the top of the robot
		 * starting from the front left wheel, the "И" will end with the back
		 * right wheel and will hit the talons in numerical order.
		 */

		front_left_wheel = new Jaguar(2);
		front_right_wheel = new Jaguar(1);
		back_left_wheel = new Jaguar(3);
		back_right_wheel = new Jaguar(0);
		shooter = new Jaguar(5);
		feeder = new Jaguar(4);
		speed = .05;

		front_right_wheel.setInverted(true);
		back_right_wheel.setInverted(true);

		// front left, back left, front right, back right
		myRobot = new RobotDrive(front_left_wheel, back_left_wheel, front_right_wheel, back_right_wheel);
		rightStick = new Joystick(1);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	public void disabledInit() {

	}

	public void disabledPeriodic() {
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
	public void autonomousInit() {

		autoSelected = (String) chooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
		autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}
	}

	public void teleopInit() {

		/*
		 * This makes sure that the autonomous stops running when teleop starts
		 * running. If you want the autonomous to continue until interrupted by
		 * another command, remove this line or comment it out.
		 */
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		double rightStickX = 0;
		double rightStickY = 0;
		double rightStickTwist = 0;
		if (Math.abs(rightStick.getX()) > thresholdX) {
			rightStickX = rightStick.getX();
		}
		if (Math.abs(rightStick.getY()) > thresholdY) {
			rightStickY = rightStick.getY();
		}
		if (Math.abs(rightStick.getTwist()) > thresholdTwist) {
			rightStickTwist = rightStick.getTwist();
		}

		// myRobot.mecanumDrive_Cartesian(-rightStickX, rightStickY,
		// rightStickTwist, 0);
		if (rightStick.getRawButton(3)) {

			if (rightStick.getRawButton(11)) {
				speed -= .01;
			}

			if (rightStick.getRawButton(12)) {
				speed += .01;
			}
		}
		if (rightStick.getTrigger()) {
			shooter.set(speed);
			feeder.set(1);

		} else {
			shooter.set(0);
			feeder.set(0);
		}

		System.out.println(speed);
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}
}
