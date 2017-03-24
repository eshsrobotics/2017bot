package org.usfirst.frc.team1759.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

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

public class Sensors {

	public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	public static BuiltInAccelerometer accel = new BuiltInAccelerometer();
}