
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
	RobotMap robotMap;
	OI oi;
	CANTalon back_right_wheel;
	CANTalon front_right_wheel;
	CANTalon back_left_wheel;
	CANTalon front_left_wheel;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		robotMap = new RobotMap();
		oi = new OI();
		
		front_left_wheel = new CANTalon(0);
		back_left_wheel = new CANTalon(1);
		front_right_wheel = new CANTalon(2);
		back_right_wheel = new CANTalon(3);
		}

	public void gyroDrive() {
		myRobot.mecanumDrive_Cartesian(RobotMap.rightStickY, RobotMap.rightStickX, RobotMap.rightStickTwist,
				robotMap.angle);
	}

	public void manualDrive() {
		myRobot.mecanumDrive_Cartesian(RobotMap.rightStickY, RobotMap.rightStickX, RobotMap.rightStickTwist, 0);
	}
}
