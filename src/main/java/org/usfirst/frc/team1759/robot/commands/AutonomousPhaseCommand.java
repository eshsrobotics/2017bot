package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.subsystems.MecanumDriveSubSystem;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.command.Command;

public class AutonomousPhaseCommand extends Command {
	private MecanumDriveSubSystem papasDrive;
	private static boolean enabled = true;
	private static int mode = 1;	// 1 = center, 2 = left, 3 = right.
	
	public AutonomousPhaseCommand(MecanumDriveSubSystem papasDrive) {
		this.papasDrive = papasDrive;
		requires(papasDrive);
	}
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}
	public void execute() {
		if(enabled) {
			if(mode == 1) {
				papasDrive.autonomousDrive(0, 0.75, 0);
				try {
					Thread.sleep(1267);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				papasDrive.autonomousDrive(0, 0, 0);
			} else if(mode == 2) {
				papasDrive.autonomousDrive(0, 0.75, 0);
				try {
					Thread.sleep(1267);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				papasDrive.autonomousDrive(0, 0, 0.25);
				try {
					Thread.sleep(1125);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				papasDrive.autonomousDrive(0, 0.75, 0);
				try {
					Thread.sleep(347);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			} else {
				papasDrive.autonomousDrive(0, 0.75, 0);
				try {
					Thread.sleep(1267);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				papasDrive.autonomousDrive(0, 0, -0.25);
				try {
					Thread.sleep(1125);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				papasDrive.autonomousDrive(0, 0.75, 0);
				try {
					Thread.sleep(347);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}
	public void stop() {
	enabled = false;
	}
}
