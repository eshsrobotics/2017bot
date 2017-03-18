package org.usfirst.frc.team1759.robot.subsystems;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.PortAssigner;
import org.usfirst.frc.team1759.robot.RobotMap;

public class BallIntake extends Subsystem{
		RobotMap robotMap;
		OI oi;
		PortAssigner portAssigner;
		CANTalon feeder;
	protected void initDefaultCommand() {
		feeder = new CANTalon(9);
	}
	public void BallIn() {
		feeder.set(.5);
	}
	public void BallOut() {
		feeder.set(-0.5);
	}
}
