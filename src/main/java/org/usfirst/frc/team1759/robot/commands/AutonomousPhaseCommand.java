package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.subsystems.MecanumDriveSubSystem;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.command.Command;

public class AutonomousPhaseCommand extends Command {
	private Joystick joystick;
	private MecanumDriveSubSystem papasDrive;
	
	public AutonomousPhaseCommand(Joystick joystick, MecanumDriveSubSystem papasDrive) {
		this.joystick = joystick;
		this.papasDrive = papasDrive;
		requires(papasDrive);
	}
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}
	public void execute() {
		papasDrive.autonomousDrive(0, .75, 0);
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		papasDrive.autonomousDrive(0, 0, 0);
	}

}
