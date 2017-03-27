package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.subsystems.MecanumDriveSubSystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousPhaseCommand extends CommandGroup {
	private MecanumDriveSubSystem papasDrive;
	private static int mode;

	public AutonomousPhaseCommand(MecanumDriveSubSystem papasDrive, int mode) {
		this.papasDrive = papasDrive;
		this.mode = mode;
		requires(papasDrive);

		if (mode == 1) {
			addSequential(new DriveCommand(papasDrive, 0, -0.75, 0), 2.5);
			// papasDrive.autonomousDrive(0, 0.75, 0);
			// try {
			// Thread.sleep(1267);
			// } catch (InterruptedException e) {
			// TODO Auto-generated catch block
			// e.printStackTrace();
			// }
			// papasDrive.autonomousDrive(0, 0, 0);
		} else if (mode == 2) {
			addSequential(new DriveCommand(papasDrive, 0, -0.75, 0), 3);
			addSequential(new DriveCommand(papasDrive, 0, 0, 0.75), 1.125);
			addSequential(new DriveCommand(papasDrive, 0, -0.75, 0), 1);
			// papasDrive.autonomousDrive(0, 0.75, 0);
			// try {
			// Thread.sleep(1267);
			// } catch (InterruptedException e) {
			// // TODO Auto-generated catch block
			// e.printStackTrace();
			// }
			// papasDrive.autonomousDrive(0, 0, 0.25);
			// try {
			// Thread.sleep(1125);
			// } catch (InterruptedException e) {
			// // TODO Auto-generated catch block
			// e.printStackTrace();
			// }
			// papasDrive.autonomousDrive(0, 0.75, 0);
			// try {
			// Thread.sleep(347);
			// } catch (InterruptedException e) {
			// // TODO Auto-generated catch block
			// e.printStackTrace();
			// }
		} else {
			addSequential(new DriveCommand(papasDrive, 0, -0.75, 0), 3);
			addSequential(new DriveCommand(papasDrive, 0, 0, -0.25), 1.125);
			addSequential(new DriveCommand(papasDrive, 0, -0.75, 0), 1);
			// papasDrive.autonomousDrive(0, 0.75, 0);
			// try {
			// Thread.sleep(1267);
			// } catch (InterruptedException e) {
			// // TODO Auto-generated catch block
			// e.printStackTrace();
			// }
			// papasDrive.autonomousDrive(0, 0, -0.25);
			// try {
			// Thread.sleep(1125);
			// } catch (InterruptedException e) {
			// // TODO Auto-generated catch block
			// e.printStackTrace();
			// }
			// papasDrive.autonomousDrive(0, 0.75, 0);
			// try {
			// Thread.sleep(347);
			// } catch (InterruptedException e) {
			// // TODO Auto-generated catch block
			// e.printStackTrace();
			// }
		}
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return isTimedOut();
	}

	public void execute() {

	}

	public void stop() {
		papasDrive.autonomousDrive(0, 0, 0);
	}
}
