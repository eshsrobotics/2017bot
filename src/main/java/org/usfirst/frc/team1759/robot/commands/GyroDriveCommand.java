
package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.RobotMap;
import org.usfirst.frc.team1759.robot.subsystems.MecanumDriveSubSystem;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GyroDriveCommand extends Command {
	RobotMap robotMap;
	OI oi;
	MecanumDriveSubSystem mecanumDriveSubSystem;

	public GyroDriveCommand() {
		// Use requires() here to declare subsystem dependencies

		requires(mecanumDriveSubSystem);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		robotMap = new RobotMap();
		oi = new OI();
		mecanumDriveSubSystem = new MecanumDriveSubSystem();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		mecanumDriveSubSystem.gyroDrive();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}