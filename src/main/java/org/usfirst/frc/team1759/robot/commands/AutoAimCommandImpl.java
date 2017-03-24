/**
 * 
 */
package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.PapasData;
import org.usfirst.frc.team1759.robot.Sensors;
import org.usfirst.frc.team1759.robot.ServerRunnable;

/**
 * Does the underlying non-WPILib-related work for the
 * {@link AutoAimBoilerCommand}.
 * 
 * @author ardonik
 */
public class AutoAimCommandImpl {

	private ServerRunnable serverRunnable;
	private boolean stopAutoAiming = false;
	private long timeOfLastVisionSolution;
	private double papasAngleOfLastVisionSolution;
	private String designatedSolutionType;
	private double initialGyroAngle;

	// we gathered this by doing this by hand the actual bot will have a
	// different rate.
	private final double ROTATION_RATE_DEGREESPERSECOND = 40;

	public AutoAimCommandImpl(ServerRunnable serverRunnable, String designatedSolutionType) {
		this.serverRunnable = serverRunnable;
		this.designatedSolutionType = designatedSolutionType;
		reset();
	}

	public void reset() {
		System.out.printf("Reset() -- Either we were just constructed or the command ended in peace.\n");
		stopAutoAiming = false;
		timeOfLastVisionSolution = -1;
		papasAngleOfLastVisionSolution = -1;
	}

	/**
	 * Returns true if the auto-aiming needs to stop.
	 * 
	 * That happens in two scenarios right now: never finding a solution in the
	 * first place, and actually reaching the target angle.
	 * 
	 * @return True if the command's isFinished() should also be true.
	 */
	public boolean isFinished() {
		return stopAutoAiming;
	}

	/**
	 * Sets us up for a new aim attempt.
	 * 
	 * @return True if we initialized successfully; false if thre is no initial
	 *         vision solution, and thus nothing to aim at.
	 */
	public boolean initialize() {
		System.out.printf("Initialize() -- This is only called when you start holding down the button.\n");

		initialGyroAngle = Sensors.gyro.getAngle();
		// Obtain the last known PapasVision solution.
		stopAutoAiming = false;
		PapasData visionSolution = serverRunnable.getPapasData();

		if (visionSolution != null) {
			papasAngleOfLastVisionSolution = visionSolution.papasAngleInDegrees;
			// System.out.printf(" Initialize(): Vision solution present.
			// SolutionFound == %s\n", visionSolution.solutionFound ? "true" :
			// "false");
		} else {
			System.out.printf("  Initialize(): Vision solution is null.  Camera client isn't talking to us?\n");
		}

		// If there's no solution, we have no target.
		if (visionSolution == null
				|| (visionSolution.solutionFound == false && visionSolution.solutionType == designatedSolutionType)) {
			stopAutoAiming = true;
			System.out.printf("Auto aim could not run because there is no %s solution \n", designatedSolutionType);
			return false;
		} else {
			return true;
		}
	}

	/**
	 * The underlying command that {@link AutoAimBoilerCommand}'s execute()
	 * method runs.
	 * 
	 * You should only call this if there has been at least one vision solution
	 * processed by the system (i.e., serverRunnable.getPapasData() != null).
	 * 
	 * `@return The amount by which to simulate a twist of the joystick in order
	 * to aim. Positive values are counterclockwise, negative values are
	 * clockwise, and 0 means we've reached our target and the aiming is done.
	 * 
	 * TODO: If we end up using the gyro, its value needs to be passed in here
	 * as a double.
	 */
	public double update() {

		// The appropriate way to do the aiming code is with a gyro. But
		// unfortunately,
		// the test-bot does not have a gyro even though the bagged bot does.
		// Thus
		// we have two separate ways to aim, and since you get an inevitable
		// COMM error
		// when deploying code that attempts to declare unused hardware.
		//
		// We're currently forced to use the less favorable approach.

		double twistValue = 0;
		PapasData currentVisionSolution = serverRunnable.getPapasData();

		if (currentVisionSolution != null && currentVisionSolution.solutionType.toString() == designatedSolutionType
				&& currentVisionSolution.papasAngleInDegrees == papasAngleOfLastVisionSolution) {
			// The vision solution is unchanged, meaning that we haven't
			// received a new solution from the client yet.
			// For this code right here, that's equivalent to having no solution
			// at all.
			currentVisionSolution = null;
		}

		// A positive number fed as the twist angle for the mecanum drive
		// will
		// currently cause the robot to turn counterclockwise. (This seems
		// to
		// be standard behavior.)
		//
		// Now, how much should we twist? Not too fast (we might overshoot
		// our
		// target), and not too slow.
		final double TWIST_SPEED_FACTOR = 0.125;

		// With the sample image 1ftH2ftD2Angle0Brightness.jpg, visual
		// inspection reveals that we need to rotate counterclockwise to
		// make the
		// papasAngle 0. Using the actual PapasVision code, we see that the
		// PapasAngle is 3.1172 degrees (probably off, but only the sign
		// matters
		// here.)
		//
		// The implication is that a counterclockwise rotation -- and
		// therefore, a
		// positive twist value -- will cause the PapasAngle to decrease.

		final double PAPAS_ANGLE_EPSILON = 5.0; // For now.

		boolean needToRotate = false;

		if (currentVisionSolution != null) {
			// Vision solution comes in pretty slowly and when it does come in
			// we need to get the most recent information.
			timeOfLastVisionSolution = System.currentTimeMillis();
			papasAngleOfLastVisionSolution = currentVisionSolution.papasAngleInDegrees;

			// If control made it here, the vision code has done its work.
			if (currentVisionSolution.solutionFound == true
					&& Math.abs(currentVisionSolution.papasAngleInDegrees) > PAPAS_ANGLE_EPSILON) {
				needToRotate = true;
			}

		} else {
			// Control only makes it here if there's no vision solution.

			double currentGyroAngle = Sensors.gyro.getAngle();
			double gyroDisplacement = currentGyroAngle - initialGyroAngle;

			// When the gyro displacement is equal to
			// -papasAngleOfLastVisionSolution, we know that we've
			// hit the last known angle of the target.
			if (Math.abs(papasAngleOfLastVisionSolution - gyroDisplacement) > PAPAS_ANGLE_EPSILON) {
				needToRotate = true;
			}
		}

		if (needToRotate) {
			if (papasAngleOfLastVisionSolution > PAPAS_ANGLE_EPSILON) {
				// Clockwise rotation.
				twistValue = -1.0 * TWIST_SPEED_FACTOR;
			} else if (papasAngleOfLastVisionSolution < -PAPAS_ANGLE_EPSILON) {
				// Counterclockwise rotation.
				twistValue = +1.0 * TWIST_SPEED_FACTOR;
			}
		} else {
			// We reached our target (or at least it's close enough.)
			twistValue = 0;
			stopAutoAiming = true;
		}

		// System.out.printf("We have %s vision solution. Twist value is %f. We
		// should%s stop auto aiming now.\n", (currentVisionSolution != null ?
		// "a" : "no"), twistValue, (stopAutoAiming == true ? "" : "n't"));
		return twistValue;
	}

}
