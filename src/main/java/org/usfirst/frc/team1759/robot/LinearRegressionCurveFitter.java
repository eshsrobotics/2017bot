package org.usfirst.frc.team1759.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * @author uakotaobi
 *
 * This class tries to calculate a line of best fit for the given set of data points.
 *
 * The algorithm was adapted from http://code.activestate.com/recipes/578129-simple-linear-regression/.
 *
 * Turns out this is the Simple Linear Regression method:
 * https://en.wikipedia.org/wiki/Simple_linear_regression.  The line of regression
 * is supposed to pass through the center of mass.  (The graph should confirm that
 * it does.)
 */
public class LinearRegressionCurveFitter extends AbstractCurveFitter {

        private Map<String, String> debugVariables = new HashMap<String, String>();

        @Override
        protected List<Double> getCoefficients(List<Coordinate> dataPoints) {

            debugVariables = new HashMap<String, String>();

            double dSum = 0, vSum = 0, vvSum = 0, dvSum = 0, ddSum = 0;
            int n = dataPoints.size();
            for (Coordinate coordinate : dataPoints) {
                dSum += coordinate.d;
                vSum += coordinate.v;
                vvSum += (coordinate.v * coordinate.v);
                dvSum += (coordinate.d * coordinate.v);
                ddSum += (coordinate.d * coordinate.d);
            }
            double denominator = Math.sqrt((ddSum - (dSum * dSum)/n) *
            		                       (vvSum - (vSum * vSum)/n));
            double r = (dvSum - (dSum * vSum)/n);

            // Calculate the slope of the line-of-best-fit.
            double m = (dvSum - (dSum * vSum)/n) / (ddSum - (dSum * dSum)/n);

            // Calculate the Y-intercept of the line-of-best-fit.
            double b = (vSum - m * dSum)/n;

            // That's it -- y = mx + b, so F(x) = mx + b, which really means that F(d) = m*d + b.
            List<Double> result = new ArrayList<Double>();
            result.add(b);
            result.add(m);

            debugVariables.put("denominator", Double.toString(denominator));
            debugVariables.put("correlationCoefficient", Double.toString(r));
            debugVariables.put("slope", Double.toString(m));
            debugVariables.put("yIntercept", Double.toString(b));
            return result;
        }

        @Override
        public Map<String, String> getDebugVariables() {
            return debugVariables;
        }
}
