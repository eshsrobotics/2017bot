
package org.usfirst.frc.team1759.robot.subsystems;

import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import java.util.Timer;

/**
 * 
 * @author Aidan Galbreath and Ari Berkowicz
 *
 *
 */
public class AutonomousDriveSubSystem extends Subsystem {

	RobotMap robotMap;
	OI oi;
	RobotDrive myRobot;
	CANTalon back_right_wheel;
	CANTalon front_right_wheel;
	CANTalon back_left_wheel;
	CANTalon front_left_wheel;
	Timer autoTimer;
	DoubleSolenoid gearSolenoid;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

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

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void autoDriveMid() {

		myRobot.mecanumDrive_Cartesian(-1, 0, 0, robotMap.angle);
		try {
			autoTimer.wait(robotMap.driveTime);
		} catch (Exception e) {
			e.printStackTrace();
		}
		myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
		gearSolenoid.set(DoubleSolenoid.Value.kForward);
	}

	public void autoDriveLeft() {

	}

	public void autoDriveRight() {

	}
}
