package org.usfirst.frc.team1759.robot;

import com.ctre.CANTalon;

/**
 * 
 */

/**
 * @author frcprogramming
 *
 */
public class Shooter {
	
	CANTalon feedWheel = null;
	CANTalon shootWheel = null;
	/**
	 * Added to use recent input from PapasData from the camera.
	 */
	private PapasData lastReceivedPapasData = null;
	
	/**
	 * 
	 * @param shoot is the Talon assignemnt to the first shooter motor.
	 * @param shoot2 is the Talon assignment to the second shooter motor.
	 */
	
	/**
	 * Constructs Talon object given the Talons assigned to each shooter motor.
	 * 
	 * @param feedWheel The wheel which, when activated, feeds a ball into the active shooting mechanism.
	 * @param shootWheel The sheel which, when activated, will launch a fed ball out into the ether.
	 */
	public Shooter(CANTalon feedWheel, CANTalon shootWheel) { 
		this.feedWheel = feedWheel;
		this.shootWheel = shootWheel;
	}
	
	/**
	 * Stores the last camera data 
	 * @param recievedPapasData used to input the last PapasData received from the camera.
	 */
	public void revievedPapasData(PapasData recievedPapasData) {
		lastReceivedPapasData = recievedPapasData;
	}
	
	/**
	 * Uses driver input to decide velocity, as opposed to deriving velocity from PapasData.
	 * @param velocity from 0 to 1 to determine shooting speed.
	 */
	public void fire(double velocity) {
		
	}
}
