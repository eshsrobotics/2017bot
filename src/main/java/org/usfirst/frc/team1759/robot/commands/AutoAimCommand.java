package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.PapasData;
import org.usfirst.frc.team1759.robot.ServerRunnable;
import org.usfirst.frc.team1759.robot.subsystems.MecanumDriveSubSystem;
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
public class AutoAimCommand extends Command {

	private MecanumDriveSubSystem mecanumDriveSubSystem;
	private ServerRunnable serverRunnable;
	private PapasData mostRecentVisionSolution;
	private boolean enabled;
	private double originalPapasAngleDegrees;
	
	/**
	 * Initialize an AutoAimCommand command object.
	 * 
	 * @param mecanumDriveSubSystem The high-level Subsystem that controls the robot's
	 *                              drive mechanism.
	 * @param serverRunnable The server thread management object that obtains vision
	 *                       solutions from the network.
	 */
	public AutoAimCommand (MecanumDriveSubSystem mecanumDriveSubSystem, ServerRunnable serverRunnable) {
		this.serverRunnable = serverRunnable;
		this.mecanumDriveSubSystem = mecanumDriveSubSystem;
		requires(mecanumDriveSubSystem);
		
		reset();
	}
	
	/**
	 * When the public start() is called to begin the AutoFireCommand, this method
	 * is called as a consequence.
	 * 
	 *  This allows us to reset our state.
	 */
	@Override
	protected void initialize() {		
		
		// Obtain the last known PapasVision solution.
		mostRecentVisionSolution = serverRunnable.getPapasData();
		
		// If there's no solution, we have no target.
		if (mostRecentVisionSolution.solutionFound == false) {
			return;
		} else {
			enabled = true;
			originalPapasAngleDegrees = mostRecentVisionSolution.papasAngleInDegrees;
		}		
	}
	
	/**
	 * If an outside party calls the public cancel() function to stop the AutoAimCommand
	 * from aiming the robot, this method gets called. 
	 */
	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		super.interrupted();
	}
	
	/**
	 * Runs this command.
	 * 
	 * This function is executed repeatedly during teleop mode until isFinished()
	 * reports that the command is done.  State must be preserved between calls. 
	 */
	@Override
	protected void execute() {
		
		// The appropriate way to do the aiming code is with a gyro.  But unfortunately,
		// the test-bot does not have a gyro even though the bagged bot does.  Thus
		// we have two separate ways to aim, and since you get an inevitable COMM error
		// when deploying code that attempts to declare unused hardware.
		//
		// We're currently forced to use the less favorable approach.
		
		final boolean usingGyro = false;		
		if (!usingGyro) {
			
			// If there's a more recent vision solution that's similar to the last
			// one we received, we'll use it.  We won't use one that's too "jumpy",
			// though -- that's probably a false positive.
			PapasData currentVisionSolution = serverRunnable.getPapasData();
			final double ACCEPTABLE_DISTANCE_DEVIANCE_INCHES = 5;
			final double ACCEPTABLE_ANGLE_DEVIANCE_DEGREES = 30;			
			
			if (currentVisionSolution.solutionFound == true &&
				Math.abs(currentVisionSolution.papasAngleInDegrees - mostRecentVisionSolution.papasAngleInDegrees) < ACCEPTABLE_ANGLE_DEVIANCE_DEGREES &&
				Math.abs(currentVisionSolution.papasDistanceInInches - mostRecentVisionSolution.papasDistanceInInches) < ACCEPTABLE_DISTANCE_DEVIANCE_INCHES) {

				mostRecentVisionSolution = currentVisionSolution;
			}															
			
			// A positive number fed as the twist angle for the mecanum drive will
			// currently cause the robot to turn counterclockwise.  (This seems to
			// be standard behavior.)
			//
			// Now, how much should we twist?  Not too fast (we might overshoot our
			// target), and not too slow.
			final double TWIST_SPEED_FACTOR = 0.5;
			
			// With the sample image 1ftH2ftD2Angle0Brightness.jpg, visual
			// inspection reveals that we need to rotate counterclockwise to make the
			// papasAngle 0.  Using the actual PapasVision code, we see that the
			// PapasAngle is 3.1172 degrees (probably off, but only the sign matters
			// here.)
			//
			// The implication is that a counterclockwise rotation -- and therefore, a
			// positive twist value -- will cause the PapasAngle to decrease.
			
			final double TWIST_EPSILON = 1.0;
			if (mostRecentVisionSolution.papasAngleInDegrees > TWIST_EPSILON) {
				
				mecanumDriveSubSystem.manualDrive(0,  0,  1.0 * TWIST_SPEED_FACTOR);
				
			} else if (mostRecentVisionSolution.papasAngleInDegrees < TWIST_EPSILON) {
				
				mecanumDriveSubSystem.manualDrive(0,  0,  -1.0 * TWIST_SPEED_FACTOR);
			
			} else {
				
				// We reached our target (or at least it's close enough.)
				enabled = false;
			}			
		}
	}
	
	/**
	 * Returns true if the command has roughly reached the vision target.
	 * @return
	 */
	@Override
	protected boolean isFinished() {
		// The moment our internal flag is tripped, we stop.
		return enabled;
	}
	
	
	/**
	 * This function is called when the command finishes peacefully (i.e., when cancel()
	 * is not called.)
	 */
	@Override
	protected void end() {
		reset();
	}
	
	private void reset() {
		this.originalPapasAngleDegrees = 0;
		this.mostRecentVisionSolution = null;
		this.enabled = false;		
	}
}
