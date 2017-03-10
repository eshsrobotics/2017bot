
package org.usfirst.frc.team1759.robot.subsystems;

import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.RobotMap;
import org.usfirst.frc.team1759.robot.PortAssigner;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;


/**
 * 
 * @author Aidan Galbreath and Ari Berkowicz
 *
 *
 */
public class AutonomousDriveSubSystem extends Subsystem {
	OI oi;
	RobotDrive myRobot;
	CANTalon back_right_wheel;
	CANTalon front_right_wheel;
	CANTalon back_left_wheel;
	CANTalon front_left_wheel;
	DoubleSolenoid gearSolenoid;
	RobotMap RobotMap;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void createTalons() {
		CANTalon talons[] = new CANTalon[10];
		for (int i = 0; i < talons.length; ++i) {
			talons[i] = new CANTalon(i);
		}
		oi = new OI();
		RobotMap = new RobotMap();

		front_left_wheel = new CANTalon(PortAssigner.getInstance().getAssignedPort(PortAssigner.LEFT_FRONT_WHEEL));
		back_left_wheel = new CANTalon(PortAssigner.getInstance().getAssignedPort(PortAssigner.LEFT_BACK_WHEEL));
		front_right_wheel = new CANTalon(PortAssigner.getInstance().getAssignedPort(PortAssigner.RIGHT_FRONT_WHEEL));
		back_right_wheel = new CANTalon(PortAssigner.getInstance().getAssignedPort(PortAssigner.RIGHT_BACK_WHEEL));
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void autoDriveMid() {

		myRobot.mecanumDrive_Cartesian(-1, 0, 0, org.usfirst.frc.team1759.robot.RobotMap.angle);
		try {
			wait(org.usfirst.frc.team1759.robot.RobotMap.driveTime);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
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
