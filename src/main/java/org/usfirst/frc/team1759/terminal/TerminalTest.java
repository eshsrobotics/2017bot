/**
 * 
 */
package org.usfirst.frc.team1759.terminal;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.usfirst.frc.team1759.robot.AbstractCurveFitter;
import org.usfirst.frc.team1759.robot.LinearRegressionCurveFitter;
import org.usfirst.frc.team1759.robot.ServerRunnable;
import org.usfirst.frc.team1759.robot.XMLParser;
import org.usfirst.frc.team1759.robot.PapasData;

/**
 * @author uakotaobi
 *
 */
public class TerminalTest {

	final static int DEFAULT_TIMEOUT_MILLISECONDS = 15000;
	
	final static Map<String, AbstractCurveFitter> algorithms = new HashMap<String, AbstractCurveFitter>();
	
	/**
	 * @param args The command-line arguments.  args[0] must be either "parse" or "listen" (for now.)  
	 */
	public static void main(String[] args) throws Exception{

		algorithms.put("linear", new LinearRegressionCurveFitter());
		
		if (args.length == 0) {
			
			System.err.println("[error] Missing sub-command; must be either 'parse' or 'listen'.");
			usage();
			
		} else {

			String subCommand = args[0].trim().toLowerCase();

			// Deal with the "parse" subcommand by feeding  the inputs into the XMLParser.
			if (subCommand.equals("parse")) {

				if (args.length == 1) {
					System.err.println("[error] Insufficient arguments to 'parse' sub-command.");
					usage();
				} else {				
					for (int i = 1; i < args.length; i++) {
						// System.out.println(String.format("[debug] XML document #%d: \"%s\"", i, args[i]));
						XMLParser parser = new XMLParser();
						PapasData data = parser.parse(args[i]);
						System.out.println(String.format("%d: %s", i, data));
					}
				}

			} else if (subCommand.equals("listen")) {
				
				int port = ServerRunnable.DEFAULT_PORT;
				int timeout = DEFAULT_TIMEOUT_MILLISECONDS;
				if (args.length > 1) {
					timeout = Integer.parseInt(args[1]);
				}
				if (args.length > 2) {
					port = Integer.parseInt(args[2]);
				}
				if (args.length > 3) {
					System.err.println("[error] Too many arguments to 'listen' sub-command.");
					usage();
				} else {

					System.out.printf("[main] Ready.\n");
					
					// Fork the thread the same way the robot would.
					ServerRunnable runnable = new ServerRunnable(port);
					Thread thread = new Thread(runnable);
					thread.setName("TerminalTest ServerRunner testing thread");
					thread.start();
					System.out.printf("[main] Spawned the ServerRunnable thread (id = %d) to listen on localhost:%d for %.2f seconds...\n", 
							thread.getId(),
							port,
							timeout / 1000.0);
					
					// Wait for the appropriate number of seconds and then make the thread die.
					Thread.sleep(timeout);
					runnable.die();
					
					// Await the thread's demise.
					System.out.printf("[main] Time's up.  Sent kill signal; awaiting thread closure....\n");
					thread.join();
					System.out.printf("[main] Thread closed.\n");
				}
				
			} else if (subCommand.equals("fit")) {
				
				if (args.length == 1) {
					System.err.println("[error] Insufficient arguments to 'fit' sub-command.");
					usage();
				} else {
					String algorithmName = args[1];
					if (!algorithms.containsKey(algorithmName)) {
						System.err.printf("[error] Invalid algorithm name \"%s\" for 'fit' sub-command.\n", algorithmName);
						usage();
					} else {

						List<Double> data = new ArrayList<Double>();
						if (args.length == 2) {
							// Come up with something random, but quadratic-ish.
							final double minDistanceInches = 0;
							final double maxDistanceInches = 60 * 12;
							final double xrange = (maxDistanceInches - minDistanceInches);
							final int numPoints = 100;
							Random generator = new Random();
							final double a = (generator.nextDouble() * 2 - 1)/1e9; // a*x^2 + b*x + c = 0
							final double b = (generator.nextDouble() * 2 - 1);
							final double c = (generator.nextDouble() * 2 - 1);
							double maxy = 0;
							for (int i = 0; i < numPoints; ++i) {
								double x = generator.nextDouble() * xrange + minDistanceInches;
								double y = a * x * x + b * x + c;
								if (y > maxy) {
									maxy = y;
								}
								double yDisplacement = (generator.nextDouble() * 0.3 - 0.15);
								data.add(x);
								data.add(y + yDisplacement);
							}

							// Postprocessing: scale the random points down to the y-range we want.
							for (int i = 0; i < numPoints; i += 2) {
								double x = data.get(i);
								double y = data.get(i + 1);
								data.set(i + i, y/maxy);
								System.out.println(String.format("echo %.2f\n", y/maxy));
							}

							System.out.println(String.format("echo %.2f*x^2 + %.2fx + %.2f = 0\n", a, b, c));
						}

						// Print the GNUPlot result.
						AbstractCurveFitter curveFitter = algorithms.get(algorithmName);
						curveFitter.setDataPoints(data);
						System.out.println(curveFitter.getPlottingCommand(String.format("Curve fitting: '%s'", algorithmName)));
					}
				}

			} else {
				System.out.printf("[debug] unrecognized sub-command \"%s\" for argument 1", subCommand);
			}
		}
	}

	/**
	 * Prints a usage message.
	 */
	public static void usage() {
		String programName = System.getProperty("sun.java.command");

		StringBuilder algorithmsList = new StringBuilder();
		for (String name : algorithms.keySet()) {
			algorithmsList.append(String.format("         %s\n", name));
		}
		
		System.out.printf(
				"Usage: %s parse XML-STRING\n\n" + "         Attempts to parse the given PapasVision XML string.\n\n"
						+ "       %s listen [TIMEOUT [PORT]]\n\n"
						+ "         Forks the ServerRunnable to listen for the given number\n"
						+ "         of milliseconds (%d by default) on the given port (%d\n"
						+ "         by default.)\n\n"
						+ "       %s fit ALGORITHM [d v [d v [...]]]\n\n"
						+ "         Calculates a curve of best fit using the given algorithm\n"
						+ "         for the given set of data points.  The algorithm choices\n"
						+ "         right now are:\n\n"
						+ "         %s\n"
						+ "         The data point are optional; if supplied, they should be\n"
						+ "         given as a list of d-v pairs, where d is a distance and v\n"
						+ "         is a velocity.  If not provided, semi-random points will be\n"
						+ "         used.\n\n"
						+ "         The output is given as a gnuplot command embedded in a Bash\n"
						+ "         here-doc.  You'll obtain gnuplot and run the command using\n"
						+ "         Bash's process substitution to generate the graph image:\n\n"
						+ "           source <(java -jar %s ...)\n\n",
				programName, 
				programName, DEFAULT_TIMEOUT_MILLISECONDS, ServerRunnable.DEFAULT_PORT, 
				programName, algorithmsList.toString(),
				programName);
	}
}
