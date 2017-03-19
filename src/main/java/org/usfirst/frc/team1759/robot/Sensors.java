package org.usfirst.frc.team1759.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.CounterBase;

/**
 *
 * @author Aidan Galbreath and Ari Berkowicz
 *
 *         This class is used to keep track of all sensors on the 2017bot, Papas
 *         Guapas (name not final). This will keep track of the gyro,
 *         accelerometer, ultrasonic, and other sensors present. This class is
 *         used similarly to the RobotMap or OI classes, as a placeholder of
 *         sorts to help with sorting.
 *
 **/

public class Sensors
{

	//public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	public static Encoder rightBack = new Encoder(6, 7, false, CounterBase.EncodingType.k2X);
	public static Encoder rightFront = new Encoder(4, 5, false, CounterBase.EncodingType.k2X);
	public static Encoder leftBack = new Encoder(2, 3, false, CounterBase.EncodingType.k2X);
	public static Encoder leftFront = new Encoder(0, 1, false, CounterBase.EncodingType.k2X);

	public static BuiltInAccelerometer accel = new BuiltInAccelerometer();

	public Ultrasonic ultraLeft = new Ultrasonic(null, null);
	public Ultrasonic ultraRight = new Ultrasonic(null, null);

}