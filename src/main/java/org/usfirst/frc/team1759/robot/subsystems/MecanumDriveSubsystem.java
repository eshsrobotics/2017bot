
package org.usfirst.frc.team1759.robot.subsystems;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Robot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/**
 * This is our drive Subsystem. Two main functions will be included in this: Gyro-reliant drive and Gyro-free drive.
 */
public class MecanumDriveSubSystem extends Subsystem {
    RobotDrive myRobot;
    Joystick rightStick;
    CANTalon back_right_wheel;
	CANTalon front_right_wheel;
	CANTalon back_left_wheel;
	CANTalon front_left_wheel;
	
	public MecanumDriveSubSystem(){
		CANTalon talons[] = new CANTalon[10];
		for (int i = 0; i < talons.length; ++i) {
			talons[i] = new CANTalon(i);
		}
		robotmap = new RobotMap();
		oi = new OI();

	    front_left_wheel = talons[0];
	    back_left_wheel = talons[1];
	    front_right_wheel = talons[2];
	    back_right_wheel = talons[3];
	}
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	


    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    public void gyroDrive() {
        myRobot.mecanumDrive_Cartesian(rightStickY, -rightStickX, -rightStickTwist, angle);
    }

    public void manualDrive() {
        myRobot.mecanumDrive_Cartesian(rightStick.getY(), rightStick.getX(), rightStick.getTwist(), 0);
    }
}

