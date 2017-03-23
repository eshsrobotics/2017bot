package org.usfirst.frc.team1759.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team1759.robot.RobotMap;

public class BallIntakeSubSystem extends Subsystem {
	RobotMap robotMap;
	static SpeedController feeder;

	public BallIntakeSubSystem(SpeedController feeder) {
		this.feeder = feeder;
	}

	protected void initDefaultCommand() {
		robotMap = new RobotMap();
	}

	public void BallIn() {
		if (feeder != null) {
			feeder.set(.5);
		}
	}

	public void BallOut() {
		if (feeder != null) {
			feeder.set(-0.5);
		}
	}

	public void stop() {
		if (feeder != null) {
			feeder.set(0);
		}
	}
}
