package org.usfirst.frc.team1759.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;

/**
 * 
 * @author Aidan Galbreath and Ari Berkowicz
 * 
 *         The RobotMap is a mapping from the ports sensors and actuators are
 *         wired into to a variable name. This provides flexibility changing
 *         wiring, makes checking the wiring easier and significantly reduces
 *         the number of magic numbers floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;
	
	//TODO: Reorganize this class, put things like Joysticks and sensors in their appropriate classes

	static ADXRS450_Gyro gyro;
	Joystick rightStick;
	
	public static final int front_left_wheel = 0;			//Port for the motor associated with our front left wheel
	public static final int back_left_wheel = 1;			//Port for the motor associated with our back left wheel
	public static final int front_right_wheel = 2;			//Port for the motor associated with our front right wheel
	public static final int back_right_wheel = 3;			//Port for the motor associated with our back right wheel
	public static final int shoot_wheel = 4;				//Port for the motor associated with our shooter
	public static final int feed_wheel = 5;					//Port for the motor associated with our feed wheel
	public static final int feeder = 6;						//Port for the motor associated with our intake motor.
	public static final double thresholdX = .35; 			// Added to make sure the drive
															// isn't too sensitive
	public static final double thresholdY = .2; 			// As Above
	public static final double thresholdTwist = .2; 		// As Above
	public static final double low = .45; 					// Added to lower speed for precision
	public static final double medium = .65; 				// Added to lower speed for power
															// saving
	public static final double high = .75; 					// Added to limit speed slightly
	public static final int max = 1; 						// Added because it made everything easier
															// to do code wise.
	public static double testShooterSpeed = .5; 			// Used to test shooter speed to
															// determine best distance.
	public static double velocity = .75;					// Shooter speed
	public static double accX = 0; 							// Acceleration in the X-direction
	public static double accY = 0; 							// Acceleration in the Y-direction
	public static double accZ = 0; 							// Acceleration in the Z-direction
	public static double accTotal = 0; 						// For making little adjustments with the
															// accelerometer code.
	public static boolean gyroIO = true;					// To toggle the gyro into manual mode
															// if necessary. 
	public static final double littleAdjust = 0.05; 		// For making little
															// adjustments.
	public static final long shooterTime = 3500;			// Time given to the shooter to build up speed
	public static final long feedTime = 1000;
	public final static double angle = gyro.getAngle();		
	public final static long driveTime = 3000;				// For Autonomous
	public static double rightStickX = 0;
	{
		if (Math.abs(rightStick.getX()) > thresholdX) {
			rightStickX = rightStick.getX();
		}
	}
	public static double rightStickY = 0;
	{
		if (Math.abs(rightStick.getY()) > thresholdY) {
			rightStickX = rightStick.getY();
		}
	}
	public static double rightStickTwist = 0;
	{
		if (Math.abs(rightStick.getX()) > thresholdTwist) {
			rightStickX = rightStick.getTwist();
		}

		// public static final ExampleSubsystem exampleSubsystem = new
		// ExampleSubsystem();

		// If you are using multiple modules, make sure to define both the port
		// number and the module. For example you with a rangefinder:
		// public static int rangefinderPort = 1;
		// public static int rangefinderModule = 1;
	}
}
