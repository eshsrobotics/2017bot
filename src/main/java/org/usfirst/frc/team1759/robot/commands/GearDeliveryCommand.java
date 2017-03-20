package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.subsystems.GearDropperSubSystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

public class GearDeliveryCommand extends Command {
	private Joystick joyStick;
	private GearDropperSubSystem gearDropperSubSystem;

	public GearDeliveryCommand(GearDropperSubSystem gearDropperSubSystem, Joystick joyStick) {
		this.joyStick = joyStick;
		this.gearDropperSubSystem = gearDropperSubSystem;
		requires(gearDropperSubSystem);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub

		if (this.joyStick.getRawButton(9) == false && this.joyStick.getRawButton(10) == false) {
			return true;
		}
		return false;
	}

	@Override
	protected void execute() {
		if (this.joyStick.getRawButton(9) == true) {
			this.gearDropperSubSystem.pullOut();
		}
		if (this.joyStick.getRawButton(10) == true) {
			this.gearDropperSubSystem.pushIn();
		}
	}
}
