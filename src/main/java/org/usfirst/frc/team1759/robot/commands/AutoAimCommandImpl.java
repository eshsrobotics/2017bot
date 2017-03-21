/**
 * 
 */
package org.usfirst.frc.team1759.robot.commands;

import org.usfirst.frc.team1759.robot.PapasData;
import org.usfirst.frc.team1759.robot.ServerRunnable;

/**
 * Does the underlying non-WPILib-related work for the {@link AutoAimCommand}.
 * 
 * @author ardonik
 */
public class AutoAimCommandImpl {

    private ServerRunnable serverRunnable;
    private PapasData mostRecentVisionSolution;
    private double originalPapasAngleDegrees;
    
    public AutoAimCommandImpl(ServerRunnable serverRunnable) {
        this.serverRunnable = serverRunnable;
        
        reset();
    }
    
    
    public void reset() {
        this.mostRecentVisionSolution = null;
        this.originalPapasAngleDegrees = 0;
    }

    /**
     * Sets us up for a new aim attempt.
     * 
     * @return True if we initialized successfully; false if thre is no initial
     *         vision solution, and thus nothing to aim at. 
     */
    public boolean initialize() {
        // Obtain the last known PapasVision solution.
        mostRecentVisionSolution = serverRunnable.getPapasData();

        // If there's no solution, we have no target.
        if (mostRecentVisionSolution == null || mostRecentVisionSolution.solutionFound == false) {
            return false;
        } else {
            originalPapasAngleDegrees = mostRecentVisionSolution.papasAngleInDegrees;
            return true;            
        }        
    }
    
    /**
     * The underlying command that {@link AutoAimCommand}'s execute() method
     * runs.
     * 
     * You should only call this if there has been at least one vision solution
     * processed by the system (i.e., serverRunnable.getPapasData() != null).
     * 
     * `@return The amount by which to simulate a twist of the joystick in order
     *          to aim.  Positive values are counterclockwise, negative values
     *          are clockwise, and 0 means we've reached our target and the
     *          aiming is done.
     *          
     *  TODO: If we end up using the gyro, its value needs to be passed in here
     *  as a double.
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

        double twistValue;
        final boolean usingGyro = false;
        if (!usingGyro) {

            // If there's a more recent vision solution that's similar to the
            // last
            // one we received, we'll use it. We won't use one that's too
            // "jumpy",
            // though -- that's probably a false positive.
            PapasData currentVisionSolution = serverRunnable.getPapasData();
            final double ACCEPTABLE_DISTANCE_DEVIANCE_INCHES = 5;
            final double ACCEPTABLE_ANGLE_DEVIANCE_DEGREES = 30;

            if (mostRecentVisionSolution == null
                    || (currentVisionSolution.solutionFound == true
                            && Math.abs(currentVisionSolution.papasAngleInDegrees -
                                        mostRecentVisionSolution.papasAngleInDegrees) < ACCEPTABLE_ANGLE_DEVIANCE_DEGREES
                            && Math.abs(currentVisionSolution.papasDistanceInInches -
                                        mostRecentVisionSolution.papasDistanceInInches) < ACCEPTABLE_DISTANCE_DEVIANCE_INCHES)) {

                mostRecentVisionSolution = currentVisionSolution;
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
            final double TWIST_SPEED_FACTOR = 0.5;

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

            final double TWIST_EPSILON = 1.0; 
            if (mostRecentVisionSolution.papasAngleInDegrees > TWIST_EPSILON) {

                twistValue = 1.0 * TWIST_SPEED_FACTOR;

            } else if (mostRecentVisionSolution.papasAngleInDegrees < TWIST_EPSILON) {

                twistValue = -1.0 * TWIST_SPEED_FACTOR;

            } else {

                // We reached our target (or at least it's close enough.)
                twistValue = 0;
            }
        }
        return twistValue;
    } 
    
}
