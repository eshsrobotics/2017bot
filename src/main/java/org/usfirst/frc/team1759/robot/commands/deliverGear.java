
package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.RobotMap;

import org.usfirst.frc.team1759.robot.subsystems.GearDropper;

import edu.wpi.first.wpilibj.command.Command;

public class deliverGear extends Command {
	RobotMap robotMap;
	OI oi;
	GearDropper gear;

	public deliverGear() {
		// Use requires() here to declare subsystem dependencies

		requires(GearDropper);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		robotMap = new RobotMap();
		oi = new OI();
		gear = new GearDropper();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		gear.pushIn();
		wait(1);
		gear.stop();
		wait(3);
		gear.pullOut();
		wait(1);
		gear.stop();
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