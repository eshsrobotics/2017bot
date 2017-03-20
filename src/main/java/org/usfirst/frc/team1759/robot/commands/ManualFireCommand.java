package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.subsystems.ShooterSubSystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

public class ManualFireCommand extends Command {
	private ShooterSubSystem shooterSubSystem;
	private Joystick joyStick;

	public ManualFireCommand(ShooterSubSystem shooterSubSystem, Joystick joyStick) {
		this.shooterSubSystem = shooterSubSystem;
		this.joyStick = joyStick;
		requires(shooterSubSystem);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub

		if (this.joyStick.getTrigger() == false) {
			return true;
		}

		return false;
	}

	@Override
	protected void execute() {
		if (this.joyStick.getTrigger() == true) {
			this.shooterSubSystem.shootManual((-this.joyStick.getThrottle() + 1) * .5);
		}
	}
}