package org.usfirst.frc.team1759.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 * 
 * @author Aidan Galbreath and Ari Berkowicz
 */

public class RobotMap {
	// TODO: Reorganize this class, put things like Joysticks and sensors in
	// their appropriate classes

	public static final int front_left_wheel = 0;
	// Port for the motor associated with our front left wheel
	public static final int back_left_wheel = 1;
	// Port for the motor associated with our back left wheel
	public static final int front_right_wheel = 2;
	// Port for the motor associated with our front right wheel
	public static final int back_right_wheel = 3;
	// Port for the motor associated with our back right wheel
	public static final int shoot_wheel = 4;
	// Port for the motor associated with our shooter
	public static final int feed_wheel = 5;
	// Port for the motor associated with our feed wheel
	public static final int feeder = 6;
	// Port for the motor associated with our intake motor.
	public static final int gearSolenoid1 = 0;
	// Port for the first port of the gear Solenoid.
	public static final int gearSolenoid2 = 0;
	// Port for the second port of the gear Solenoid.
	public static final double thresholdX = 0;
	// Added to make sure the drive isn't too sensitive
	public static final double thresholdY = 0;
	// As Above
	public static final double thresholdTwist = 0;
	// As Above
	public static final double low = .45;
	// Added to lower speed for precision
	public static final double medium = .65;
	// Added to lower speed for power saving
	public static final double high = .75;
	// Added to limit speed slightly
	public static final int max = 1;
	// Added because it made everything easier to do code wise.
	public static double testShooterSpeed = .5;
	// Used to test shooter speed to determine best distance.
	public static double velocity = .75;
	// Shooter speed
	public static double accX = 0;
	// Acceleration in the X-direction
	public static double accY = 0;
	// Acceleration in the Y-direction
	public static double accZ = 0;
	// Acceleration in the Z-direction
	public static double accTotal = 0;
	// For making little adjustments with the accelerometer code.
	public static boolean gyroIO = false;
	// To toggle the gyro into manual mode if necessary.
	public static final double littleAdjust = 0.05;
	/**
	 * The amount of time we wait for the shoot wheel to accelerate to the
	 * desired speed before activating the feed wheel.
	 * 
	 * Shooting a ball before the shoot wheel has fully accelerated leads to
	 * inconsistent firing performance, which we want to avoid as much as we
	 * can.
	 */
	public static final long shootWheelRampUpTimeMilliseconds = 10;

	/**
	 * The amount of time that the feed wheel remains active is effectively the
	 * amount of time that our shooter's "trigger" is held down -- our "burst
	 * time."
	 */
	public static final long feedWheelBurstTimeMilliseconds = 100;

	public final static double angle = 0;
	public final static long driveTime = 3000;
	// For Autonomous
}
