
package org.usfirst.frc.team1759.robot.subsystems;

import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.PortAssigner;
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
	OI oi;
	RobotMap robotMap;
	boolean enabled;
	SpeedController back_right_wheel;
	SpeedController front_right_wheel;
	SpeedController back_left_wheel;
	SpeedController front_left_wheel;
	public MecanumDriveSubSystem(SpeedController back_right_wheel, SpeedController front_right_wheel, SpeedController back_left_controller, SpeedController front_left_wheel) {
		this.back_right_wheel = back_right_wheel;
		this.front_right_wheel = front_right_wheel;
		this.back_left_wheel = back_left_controller;
		this.front_left_wheel = front_left_wheel;
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		oi = new OI();
		robotMap = new RobotMap();	
	}

	public void gyroDrive() {
		myRobot.mecanumDrive_Cartesian(RobotMap.rightStickY, RobotMap.rightStickX, RobotMap.rightStickTwist,
				RobotMap.angle);
	}

	public void manualDrive() {
		myRobot.mecanumDrive_Cartesian(RobotMap.rightStickY, RobotMap.rightStickX, RobotMap.rightStickTwist, 0);
	}
}
