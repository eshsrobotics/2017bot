package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.PapasData;
import org.usfirst.frc.team1759.robot.ServerRunnable;
import org.usfirst.frc.team1759.robot.subsystems.MecanumDriveSubSystem;
import edu.wpi.first.wpilibj.command.Command;

/**
 * A command that uses the gyro, the latest PapasData vision solutions, and the
 * shooter to automatically fire the balls at the Boiler vision target *if* the
 * target is in range.
 * 
 * @author frcprogramming
 *
 */
public class AutoAimBoilerCommand extends Command {

	private MecanumDriveSubSystem mecanumDriveSubSystem;
	private boolean enabled;
	private AutoAimCommandImpl impl;

	/**
	 * Initialize an AutoAimCommand command object.
	 * 
	 * @param mecanumDriveSubSystem
	 *            The high-level Subsystem that controls the robot's drive
	 *            mechanism.
	 * @param serverRunnable
	 *            The server thread management object that obtains vision
	 *            solutions from the network.
	 */
	public AutoAimBoilerCommand(MecanumDriveSubSystem mecanumDriveSubSystem, ServerRunnable serverRunnable) {
		impl = new AutoAimCommandImpl(serverRunnable, "Boiler");
		this.mecanumDriveSubSystem = mecanumDriveSubSystem;
		requires(mecanumDriveSubSystem);

		reset();
	}

	/**
	 * When the public start() is called to begin the AutoFireCommand, this
	 * method is called as a consequence.
	 * 
	 * This allows us to put ourselves in the initial state.
	 */
	@Override
	protected void initialize() {
		enabled = impl.initialize();
	}

	/**
	 * If an outside party calls the public cancel() function to stop the
	 * AutoAimCommand from aiming the robot, this method gets called.
	 */
	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		super.interrupted();
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

		double twistValue = impl.update();
		mecanumDriveSubSystem.manualDrive(0, 0, twistValue);

	}

	/**
	 * Returns true if the command has roughly reached the vision target.
	 * 
	 * @return
	 */
	@Override
	protected boolean isFinished() {
		return impl.isFinished();
	}

	/**
	 * This function is called when the command finishes peacefully (i.e., when
	 * cancel() is not called.)
	 */
	@Override
	protected void end() {
		reset();
	}

	private void reset() {
		impl.reset();
		this.enabled = false;
	}
}
