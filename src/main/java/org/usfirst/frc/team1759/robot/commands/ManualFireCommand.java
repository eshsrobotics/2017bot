package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.RobotMap;
import org.usfirst.frc.team1759.robot.subsystems.ShooterSubSystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

public class ManualFireCommand extends Command {
	private ShooterSubSystem shooterSubSystem;
	private Joystick joyStick;
	RobotMap robotMap;

	public ManualFireCommand(ShooterSubSystem shooterSubSystem, Joystick joyStick) {
		this.shooterSubSystem = shooterSubSystem;
		this.joyStick = joyStick;
		this.robotMap = new RobotMap();
		requires(shooterSubSystem);
	}

	@Override
	protected void initialize() {
		shooterSubSystem.startShooting(getVelocityFromJoystick());
	}
	
	@Override 
	protected void end() {
		shooterSubSystem.stopShooting();
	}
	
	@Override
	protected boolean isFinished() {
		if (this.joyStick.getTrigger() == false) {
			return true;
		}

		return false;
	}

	/**
	 * The joystick has a getThrottle() method that returns a value between -1 and 1.
	 * We scale this into a value between 0 and 1 to return.
	 */
	private double getVelocityFromJoystick() {
		return (-this.joyStick.getThrottle() + 1) * .5;
	}
}