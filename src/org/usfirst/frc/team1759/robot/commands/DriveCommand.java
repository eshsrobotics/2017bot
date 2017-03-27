package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.subsystems.MecanumDriveSubSystem;

import edu.wpi.first.wpilibj.command.Command;

public class DriveCommand extends Command {

	MecanumDriveSubSystem papasDrive;

	double veloX;
	double veloY;
	double veloTwist;

	public DriveCommand(MecanumDriveSubSystem papasDrive, double veloX, double veloY, double veloTwist) {
		this.papasDrive = papasDrive;
		this.veloX = veloX;
		this.veloY = veloY;
		this.veloTwist = veloTwist;
		requires(papasDrive);
	}

	public void execute() {
		papasDrive.autonomousDrive(veloX, veloY, veloTwist);
	}

	@Override
	public void end() {
		papasDrive.autonomousDrive(0, 0, 0);
	}

	@Override
	protected boolean isFinished() {
		return isTimedOut();
	}

}
