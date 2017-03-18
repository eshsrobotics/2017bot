package org.usfirst.frc.team1759.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.PortAssigner;
import org.usfirst.frc.team1759.robot.RobotMap;

public class BallIntake extends Subsystem{
		RobotMap robotMap;
		OI oi;
		PortAssigner portAssigner;
		static SpeedController feeder;
		public BallIntake(SpeedController feeder) {
			this.feeder = feeder;
		}
	protected void initDefaultCommand() {
		robotMap = new RobotMap();
		oi = new OI();
	}
	public void BallIn() {
		feeder.set(.5);
	}
	public void BallOut() {
		feeder.set(-0.5);
	}
	public void stop() {
		feeder.set(0);
	}
}
