package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.PapasData;
import org.usfirst.frc.team1759.robot.ServerRunnable;
import org.usfirst.frc.team1759.robot.subsystems.ShooterSubSystem;

import edu.wpi.first.wpilibj.command.Command;

/**
 * A command that uses the gyro, the latest PapasData vision solutions, and the
 * shooter to automatically fire the balls at the Boiler vision target *if* the
 * target is in range.
 * 
 * @author frcprogramming
 *
 */
public class AutoFireCommand extends Command {

	private ShooterSubSystem shooterSubSystem;
	private ServerRunnable serverRunnable;
	private PapasData mostRecentVisionSolution;
	private boolean enabled;

	/**
	 * Initialize an AutoFireCommand command object.
	 * 
	 * @param shooterSubSystem
	 *            The high-level Subsystem that controls the robot's shooting
	 *            mechanism.
	 * @param serverRunnable
	 *            The server thread management object that obtains vision
	 *            solutions from the network.
	 */
	public AutoFireCommand(ShooterSubSystem shooterSubSystem, ServerRunnable serverRunnable) {
		this.serverRunnable = serverRunnable;
		this.shooterSubSystem = shooterSubSystem;
		requires(shooterSubSystem);

		reset();
	}

	/**
	 * When the public start() is called to begin the AutoFireCommand, this
	 * method is called as a consequence.
	 * 
	 * This allows us to reset our state.
	 */
	@Override
	protected void initialize() {
		this.shooterSubSystem.startShooting();
	}

	/**
	 * If an outside party calls the public cancel() function to stop the
	 * AutoFireCommand from aiming the robot, this method gets called.
	 */
	@Override
	protected void interrupted() {
		this.shooterSubSystem.stopShooting();
	}

	/**
	 * Runs this command.
	 * 
	 * This function is executed repeatedly during teleop mode until
	 * isFinished() reports that the command is done. State must be preserved
	 * between calls.
	 */
	@Override
	protected void execute() {
		this.shooterSubSystem.updateVelocity();
	}

	/**
	 * Returns true if the command has roughly reached the vision target.
	 * 
	 * @return
	 */
	@Override
	protected boolean isFinished() {
		// The moment our internal flag is tripped, we stop.
		if (!enabled) {
			return false;
		}
		return true;
	}

	/**
	 * This function is called when the command finishes peacefully (i.e., when
	 * cancel() is not called.)
	 */
	@Override
	protected void end() {
		reset();
		this.shooterSubSystem.stopShooting();
	}

	private void reset() {
		this.mostRecentVisionSolution = null;
		this.enabled = false;
	}
}
