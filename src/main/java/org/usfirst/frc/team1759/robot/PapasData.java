package org.usfirst.frc.team1759.robot;

/**
 * A struct that holds the data we get back from PapasVision in an easy-to-digest format. 
 * 
 * It's not a big deal.  It's not managed.  It's just a tuple.
 */
public class PapasData {	
	public double papasDistanceInInches = -1;
	public double papasAngleInDegrees = -1;
	public boolean solutionFound = false;
	public String solutionType = "";
	public String toString() {
		return String.format("[Solution found: %s], [Solution type: %s], [Distance: %.2f in], [Angle: %.2f deg]", solutionFound, solutionType, papasDistanceInInches, papasAngleInDegrees);
	}
}