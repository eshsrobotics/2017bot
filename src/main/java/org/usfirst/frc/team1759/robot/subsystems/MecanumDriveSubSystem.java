
package org.usfirst.frc.team1759.robot.subsystems;

import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This is our drive Subsystem. Two main functions will be included in this:
 * Gyro-reliant drive and Gyro-free drive.
 */
public class MecanumDriveSubSystem extends Subsystem {
	RobotDrive myRobot;
	CANTalon back_right_wheel;
	CANTalon front_right_wheel;
	CANTalon back_left_wheel;
	CANTalon front_left_wheel;
	RobotMap robotMap;
	OI oi;

	public void createTalons() {
		CANTalon talons[] = new CANTalon[10];
		for (int i = 0; i < talons.length; ++i) {
			talons[i] = new CANTalon(i);
		}
		robotMap = new RobotMap();
		oi = new OI();

		front_left_wheel = talons[0];
		back_left_wheel = talons[1];
		front_right_wheel = talons[2];
		back_right_wheel = talons[3];
	}

	public MecanumDriveSubSystem() {
		
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		createTalons();
	}

	public void gyroDrive() {
		myRobot.mecanumDrive_Cartesian(RobotMap.rightStickY, RobotMap.rightStickX, RobotMap.rightStickTwist,
				robotMap.angle);
	}

	public void manualDrive() {
		myRobot.mecanumDrive_Cartesian(RobotMap.rightStickY, RobotMap.rightStickX, RobotMap.rightStickTwist, 0);
	}
}
