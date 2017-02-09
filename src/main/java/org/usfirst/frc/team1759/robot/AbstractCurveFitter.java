/**
 * 
 */
package org.usfirst.frc.team1759.robot;

import java.util.ArrayList;
import java.util.List;

import org.omg.CORBA.portable.ApplicationException;

/**
 * @author uakotaobi
 *
 * An abstraction for the concept of a curve fitting analyzer.
 * 
 * Curve fitters take a set of data points in (Distance, Velocity) space (where 
 * 0 < Distance and 0 <= Velocity <= 1) and calculate the coefficients of polynomial
 * that best fits those data points.  The polynomial can subsequently be used to predict
 * the proper velocity for a given distance, or simply plotted on a graph. 
 */
public abstract class AbstractCurveFitter {

	/**
	 * A tiny struct that holds (distance, velocity) pairs.
	 */
	protected class Coordinate {
		public Coordinate(double d, double v) { this.d = d; this.v = v; }
		
		/**
		 * The distance (from the camera target) this coordinate represents, in inches.
		 */
		public double d;
		
		/**
		 * The shooting velocity this coordinate represents, ranging from 0 for minimum power
		 * to 1 for maximum power.
		 */
		public double v;
	}
	
	/**
	 * The data that the curve fitters will analyze. 
	 */
	protected List<Coordinate> data;
	
	/**
	 * Sets the data points to use for curve fitting. 
	 * 
	 * @param dataPoints A list of length 2*N, where N is the number of data points to analyze.  The even indices
	 *                   of the array, starting with element 0, and the distance coordinates.  The odd indices of
	 *                   the array, starting with 1, are velocity coordinates.
	 *
	 *                   If the length of the array is an odd number, the last element will be ignored. 
	 */
	public void setDataPoints(List<Double> dataPoints) {
		data = new ArrayList<Coordinate>();
		for (int i = 0; i < (dataPoints.size() - dataPoints.size() % 2); i += 2) {
			data.add(new Coordinate(dataPoints.get(i), dataPoints.get(i + 1)));
		}
	}
	
	/**
	 * Gets the coefficients of the polynomial curve that best fits the given data.
	 */
	public List<Double> getCoefficients() {
		return getCoefficients(data);
	}
	
	/**
	 * Gets the GNUPlot command to graph both the set of data points and the curve that best fits them.
	 * 
	 * @param title The title of the graph.
	 *  
	 * @return A string which can be executed to create a graph image called "result.png" in the current 
	 *         directory.  You will require your local system's gnuplot package. 
	 */
	public String getPlottingCommand(String title) {

		// Max distance on the field (I'm told it's 60 feet long.)
		final int MAX_DISTANCE_INCHES = 60 * 12; 

		// The more samples. the smoother the graph.
		final int MAX_SAMPLES = 40;

		// Where to save the fancy graph when the command is run.
		final String OUTPUT_FILE = "./result.png";

		StringBuilder stringBuilder = new StringBuilder();
		stringBuilder.append("gnuplot <<- EOF\n");
		stringBuilder.append("  set term png\n");
		stringBuilder.append(String.format("  set output \"%s\"\n", OUTPUT_FILE));
		stringBuilder.append(String.format("  set title \"%s\"\n", title));
		stringBuilder.append(String.format("  set xrange[0:%d]\n", MAX_DISTANCE_INCHES));
		stringBuilder.append("  set xlabel \"Distance (inches)\"\n");
		stringBuilder.append("  set yrange[0:1]\n");
		stringBuilder.append("  set ylabel \"Motor velocity (%)\"\n");
		stringBuilder.append("  set autoscale\n");
		stringBuilder.append("  set style line 1 linewidth 5\n");
		stringBuilder.append(String.format("  set samples %d\n", MAX_SAMPLES));

		// Unfortunately, to enter inline data for the plot, we have to use newlines.
		stringBuilder.append("  plot \"-\" title \"Raw Data\" with dots, \"-\" title \"Fitting Curve\" with lines\n");

		// Inline data for the scatter plot ("dots").
		for (Coordinate c : data) {
			stringBuilder.append(String.format("  %f %f\n", c.d, c.v));
		}
		stringBuilder.append("  e\n");

		// Inline data for the curve itself ("lines").
		for (double d = 0; d < MAX_DISTANCE_INCHES; d += (double)MAX_DISTANCE_INCHES/MAX_SAMPLES) {
			double v = evaluate(d);
			stringBuilder.append(String.format("  %f %f\n", d, v));
		}
		stringBuilder.append("  e\n");
		stringBuilder.append("EOF\n");

		return stringBuilder.toString();
	}


	/**
	 * Evaluates the polynomial at the given distance coordinate and returns the corresponding velocity.
	 * Calls {@link getCoefficients} to do its dirty work -- after that, it's just multiplying and adding.
	 *  
	 * @param d The distance (in inches.)
	 * @return The velocity (ranging from 0 to 1.)
	 */
	public double evaluate(double d) {
		
		List<Double> coefficients = getCoefficients();
		
		// This is Horner's method.
		int index = coefficients.size() - 1;
		double result = coefficients.get(coefficients.size() - 1);
		for (int i = index - 1; i >= 0; --i) {
			result = coefficients.get(i) + result * d;
		}
		return result;
	}
	
	/**
	 * Gets the coefficients of the polynomial curve v = F(d) that best fits the given data.
	 * 
	 * Must be implemented by derived classes.
	 * 
	 * @param dataPoints A list of data points.
	 * @return The coefficients of the polynomial, starting with the coefficient for the term with the highest
	 *         degree.
	 */ 
	protected abstract List<Double> getCoefficients(List<Coordinate> dataPoints); 
}
