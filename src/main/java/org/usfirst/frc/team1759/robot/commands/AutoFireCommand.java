package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.subsystems.ShooterSubSystem;

import edu.wpi.first.wpilibj.command.Command;

/**
 * A command that uses the gyro, the latest PapasData vision solutions,
 * and the shooter to automatically fire the balls at the Boiler
 * vision target *if* the target is in range.
 *  
 * @author frcprogramming
 *
 */
public class AutoFireCommand extends Command {

	private ShooterSubSystem shooterSubSystem;
	
	public AutoFireCommand (ShooterSubSystem shooterSubSystem) {
		this.shooterSubSystem = shooterSubSystem;
		requires(shooterSubSystem);		
	}
	
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

}
