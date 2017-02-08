package org.usfirst.frc.team1759.robot;

import com.ctre.CANTalon;

/**
 * @author frcprogramming
 *
 * This class manages the motors to the shooting mechanism for the 2017bot.  It knows how to fire one ball at a time
 * at the given speed, and has some smarts to allow it to fir just far enough to hit the target at the given
 * papasDistance.
 */
public class Shooter {
	
	private CANTalon feedWheel = null;
	private CANTalon shootWheel = null;
	/**
	 * Added to use recent input from PapasData from the camera.
	 */
	private PapasData lastReceivedPapasData = null;
		
	/**
	 * Constructs Talon object given the Talons assigned to each shooter motor.
	 * 
	 * @param feedWheel The wheel which, when activated, feeds a ball into the active shooting mechanism.
	 *                  It should already have been initialized and assigned to a port. 
	 * @param shootWheel The wheel which, when activated, will launch a fed ball out into the ether.  It
	 *                   should already have been initialized and assigned to a port as well.
	 */
	public Shooter(CANTalon feedWheel, CANTalon shootWheel) { 
		this.feedWheel = feedWheel;
		this.shootWheel = shootWheel;
	}
	
	/**
	 * Stores the last camera data 
	 * @param recievedPapasData used to input the last PapasData received from the camera.
	 */
	public void receivedPapasData(PapasData recievedPapasData) {
		lastReceivedPapasData = recievedPapasData;
	}
	
	/**
	 * Fires a ball.  Uses driver input to decide velocity, as opposed to deriving velocity from PapasData.
	 * 
	 * TODO: Implement this.
	 * 
	 * @param velocity from 0 to 1 to determine shooting speed.
	 */
	public void fire(double velocity) {

		// Order of events:
		// (1) Activate feedwheel to load one ball
		// (2) Shut down feedwheel
		// (3) Activate shootwheel to fire the loaded ball
		// (4) Deactivate shootsheel.
		//
		// Consistency is key; apart from defects in the flight path due to initial orientation and wind resistance,
		// we want shooting with the same velocity to have results that are as reproducible as possible.
	}
	
	/**
	 * Fires a ball.  Uses the last {@link receivedPapasData} to determine the velocity by means of our curve-fitting function.
	 * 
	 * Calls {@link fire(double)} once the velocity has been determined.
	 * 
	 * TODO: Implement this.
	 */
	public void fire() {
	}
	
	/**
	 * A function that uses a polynomial curve-fitting function to interpolate the ideal velocity to fire the ball
	 * at, given our (camera's) distance from the target('s reflective tape.)
	 * 
	 * TODO: Gather the data to derive a curve for this.
	 * 
	 * @param distanceInInches Nominally {@link this.lastReceivedPapasData.papasDistance}, though you can pass any number
	 *                         you like in here.  If the distance is less than or equal to 0, we'll return 0 and make fun
	 *                         of you in the logs. 
	 * @return A velocity value to feed to the firing wheel.  The velocity will always be in the closed interval [0, 1],
	 *         where 0 is minimum velocity and 1 is maximum velocity. 
	 */
	private double convertPapasDistanceToVelocity(double distanceInInches) {
		return 0;
	}
}
