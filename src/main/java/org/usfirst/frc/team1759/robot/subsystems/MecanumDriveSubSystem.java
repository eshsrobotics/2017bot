
package org.usfirst.frc.team1759.robot.subsystems;

import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * 
 * @author Aidan Galbreath and Ari Berkowicz This is our drive Subsystem. Two
 *         main functions will be included in this: Gyro-reliant drive and
 *         Gyro-free drive.
 */
public class MecanumDriveSubSystem extends Subsystem {
	RobotDrive myRobot;
	RobotMap robotMap;
	boolean enabled;
	SpeedController back_right_wheel;
	SpeedController front_right_wheel;
	SpeedController back_left_wheel;
	SpeedController front_left_wheel;

	/*
	 * If you draw an imaginary "И" (Cyrillic ee) on the top of the robot
	 * starting from the front left wheel, the "И" will end with the back right
	 * wheel and will hit the talons in numerical order.
	 */

	public MecanumDriveSubSystem(SpeedController back_right_wheel, SpeedController front_right_wheel,
			SpeedController back_left_controller, SpeedController front_left_wheel) {
		this.back_right_wheel = back_right_wheel;
		this.front_right_wheel = front_right_wheel;
		this.back_left_wheel = back_left_controller;
		this.front_left_wheel = front_left_wheel;

		front_right_wheel.setInverted(true);
		back_right_wheel.setInverted(true);
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		robotMap = new RobotMap();
		myRobot = new RobotDrive(front_left_wheel, back_left_wheel, back_right_wheel, front_right_wheel);
	}

	public void gyroDrive(double joyStickY, double joyStickX, double joyStickTwist) {
		myRobot.mecanumDrive_Cartesian(joyStickY, joyStickX, joyStickTwist, RobotMap.angle);
	}

	public void manualDrive(double joyStickX, double joyStickY, double joyStickTwist) {
		myRobot.mecanumDrive_Cartesian(joyStickX, -joyStickY, joyStickTwist, 0);
	}
	public void haloDrive(double joyStickX, double joyStickY, double joyStickTwist){
		myRobot.mecanumDrive_Cartesian(joyStickX, -joyStickY, joyStickTwist, 0);
	}
}
