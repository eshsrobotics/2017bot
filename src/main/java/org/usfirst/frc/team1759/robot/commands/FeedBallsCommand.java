package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.subsystems.BallIntakeSubSystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

public class FeedBallsCommand extends Command {
	private Joystick joyStick;
	private BallIntakeSubSystem ballInTakeSubSystem;

	public FeedBallsCommand(BallIntakeSubSystem ballInTakeSubSystem, Joystick joyStick) {
		this.ballInTakeSubSystem = ballInTakeSubSystem;
		this.joyStick = joyStick;
		requires(ballInTakeSubSystem);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub

		if (this.joyStick.getRawButton(7) == false && this.joyStick.getRawButton(8) == false) {
			return true;
		}
		return false;
	}

	@Override
	protected void execute() {
		if (this.joyStick.getRawButton(8) == true) {
			this.ballInTakeSubSystem.BallIn();
		}
		if (this.joyStick.getRawButton(7) == true) {
			this.ballInTakeSubSystem.BallOut();
		}
	}
}
