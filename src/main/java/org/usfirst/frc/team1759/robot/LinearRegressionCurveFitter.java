/**
 * 
 */
package org.usfirst.frc.team1759.robot;

import java.util.ArrayList;
import java.util.List;

/**
 * @author uakotaobi
 *
 * This class tries to calculate a line of best fit for the given set of data points.
 * 
 * The formula came from the first few seconds of this Khan Academy video:
 * https://www.youtube.com/watch?v=GAmzwIkGFgE
 */
public class LinearRegressionCurveFitter extends AbstractCurveFitter {

	/**
	 * Does what you think it does.
	 * 
	 * @param args A variable list of one or more doubles to average.
	 * @return The arithmetic mean, or 0 if no args were provided.
	 */
	private static double average(double... args) {
		if (args.length == 0) {
			return 0;
		}
		
		double sum = 0;
		for (double value : args) {
			sum += value;
		}
		return sum / args.length;
	}
	
	@Override
	protected List<Double> getCoefficients(List<Coordinate> dataPoints) {

		double dSum = 0, vSum = 0, dvSum = 0, dSquaredSum = 0;
		for (Coordinate coordinate : dataPoints) {
			dSum += coordinate.d;
			vSum += coordinate.v;
			dvSum += (coordinate.d * coordinate.v);
			dSquaredSum += (coordinate.d * coordinate.d);
		}
		double dAverage = dSum/dataPoints.size();
		double vAverage = vSum/dataPoints.size();
		double dvAverage = dvSum/dataPoints.size();
		double dSquaredAverage = dSquaredSum/dataPoints.size();
		
		// Calculate the slope of the line-of-best-fit.
		double m = (dAverage * vAverage - dvAverage) / (dAverage * dAverage - dSquaredAverage); 
		
		// Calculate the Y-intercept of the line-of-best-fit.
		double b = vAverage - m * dAverage;
		
		// That's it -- y = mx + b, so F(x) = mx + b, which really means that F(d) = m*d + b. 		
		List<Double> result = new ArrayList<Double>();
		result.add(m);
		result.add(b);
		return result;
	}

}
