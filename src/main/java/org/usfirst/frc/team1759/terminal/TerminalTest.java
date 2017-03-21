/**
 *
 */
package org.usfirst.frc.team1759.terminal;

import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.usfirst.frc.team1759.robot.PapasData;
import org.usfirst.frc.team1759.robot.ServerRunnable;
import org.usfirst.frc.team1759.robot.XMLParser;
import org.usfirst.frc.team1759.robot.AbstractCurveFitter;
import org.usfirst.frc.team1759.robot.LinearRegressionCurveFitter;
import org.usfirst.frc.team1759.robot.commands.AutoAimCommand;
import org.usfirst.frc.team1759.robot.commands.AutoAimCommandImpl;
import org.usfirst.frc.team1759.robot.subsystems.MecanumDriveSubSystem;

/**
 * @author uakotaobi
 *
 */
public class TerminalTest {

    /**
     * The amount of time that the 'listen' sub-command listens before signaling
     * the server thread to quit.
     */
    final static int DEFAULT_TIMEOUT_MILLISECONDS = 15000;

    /**
     * The group of (name, curve fitter) pairs that the 'fit' sub-command
     * recognizes.
     */
    final static Map<String, AbstractCurveFitter> algorithms = new HashMap<String, AbstractCurveFitter>();

    /**
     * @param args
     *            The command-line arguments. args[0] is the sub-command.
     */
    public static void main(String[] args) throws Exception {

        algorithms.put("linear", new LinearRegressionCurveFitter());

        if (args.length == 0) {

            System.err
                    .println("[error] Missing sub-command; must be either, 'parse', 'listen', or 'fit'.");
            usage();

        } else {

            String subCommand = args[0].trim().toLowerCase();

            // Deal with the "parse" subcommand by feeding the inputs into the
            // XMLParser.
            if (subCommand.equals("parse")) {

                if (args.length == 1) {
                    System.err
                            .println("[error] Insufficient arguments to 'parse' sub-command.");
                    usage();
                } else {
                    for (int i = 1; i < args.length; i++) {
                        // System.out.println(String.format("[debug] XML document #%d: \"%s\"",
                        // i, args[i]));
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
                    System.err
                            .println("[error] Too many arguments to 'listen' sub-command.");
                    usage();
                } else {

                    System.out.printf("[main] Ready.\n");

                    // Fork the thread the same way the robot would.
                    ServerRunnable runnable = new ServerRunnable(port);
                    Thread thread = new Thread(runnable);
                    thread.setName("TerminalTest ServerRunner testing thread");
                    thread.start();
                    System.out.printf("[main] Spawned the ServerRunnable thread (id = %d) to listen on localhost:%d for %.2f seconds...\n",
                                      thread.getId(), port, timeout / 1000.0);

                    // The server's forked. It's our time to shine!

                    AutoAimCommandImpl autoAimCommandImpl = new AutoAimCommandImpl(runnable);
                    long startTime = System.currentTimeMillis();
                    double currentTwist = 999999999;

                    // We don't care if the initialization fails; we're going to
                    // pretend we can still aim with this command anyway.
                    autoAimCommandImpl.initialize();
                    
                    while (System.currentTimeMillis() - startTime < timeout) {
                        // The aim command will be sending rotation commands
                        // to the drive, but our version of the drive isn't
                        // connected to any actual SpeedControllers and
                        // doesn't do anything. However, the
                        // AutoAimCommand was deliberately written to expose
                        // some of its internal variables so that we can
                        // check to see how it's aiming.
                        //
                        // The "robot" doesn't actually rotate, so all we can
                        // do is print the direction we would have told the
                        // drive to go. Try sending multiple XMl updates to
                        // the server if you wish to simulate more realistic
                        // rotation.

                        if (runnable.getPapasData() != null) {
                            double lastKnownTwist = currentTwist;
                            currentTwist = autoAimCommandImpl.update();
    
                            // Only print something if the situation has changed.
                            if (lastKnownTwist != currentTwist) {
                                if (currentTwist == 0) {
                                    System.out.printf("  * AutoAimCommandImpl reports that it has reached its target (isFinished() == true).\n");
                                } else {
                                    System.out.printf("  * AutoAimCommandImpl reports that it has rotated %s (twist == %f).\n",
                                                      (currentTwist > 0 ? "counterclockwise" : "clockwise"),
                                                      currentTwist);
                                }
                            }
                        }

                        Thread.yield();
                    }

                    // Time's up.  Kill the server thread.
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
                    return;
                }

                String algorithmName = args[1];
                if (!algorithms.containsKey(algorithmName)) {
                    System.err.printf("[error] Invalid algorithm name \"%s\" for 'fit' sub-command.\n",
                                      algorithmName);
                    usage();
                    return;
                }
                ;

                List<Double> data = new ArrayList<Double>();
                if (args.length == 2) {

                    // The user supplied no data.
                    // Come up with something random, but quadratic-ish.
                    final double minDistanceInches = 0;
                    final double maxDistanceInches = 60 * 12;
                    final double xrange = (maxDistanceInches - minDistanceInches);
                    final int numPoints = 100;

                    Random generator = new Random();
                    final double scaleFactor = 1e-4;
                    final double a = generator.nextDouble(); // a*x^2 + b*x + c
                                                             // = 0, a always
                                                             // positive
                    final double b = (generator.nextDouble() * 2 - 1);
                    final double c = (generator.nextDouble() * 2 - 1);

                    for (int i = 0; i < numPoints; ++i) {
                        double x = generator.nextDouble() * xrange
                                + minDistanceInches;
                        double y = a * x * x + b * x + c;
                        double yDisplacement = (generator.nextDouble() * 2 - 1)
                                / scaleFactor;
                        data.add(x);
                        data.add((y + yDisplacement) * scaleFactor);
                    }

                } else {

                    // Use the data that the user gave us on the command line.
                    for (int i = 2; i < args.length; ++i) {
                        data.add(Double.parseDouble(args[i]));
                    }
                }

                // Print the GNUPlot result.
                AbstractCurveFitter curveFitter = algorithms.get(algorithmName);
                curveFitter.setDataPoints(data);
                System.out.println(curveFitter.getPlottingCommand(String
                        .format("Curve fitting: '%s' (%s)", algorithmName,
                                curveFitter.toString())));

                // Print out the debug variables.
                Map<String, String> debugVariables = curveFitter
                        .getDebugVariables();
                for (String name : debugVariables.keySet()) {
                    System.out.printf("echo '[debug] %s: %s'\n", name,
                            debugVariables.get(name));
                }

            } else {
                System.out.printf("[debug] unrecognized sub-command \"%s\" for argument 1",
                                  subCommand);
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
            algorithmsList.append(String.format("            * %s\n", name));
        }

        System.out.printf("Usage: %s parse XML-STRING\n\n"
                          + "         Attempts to parse the given PapasVision XML string.\n\n"
                          + "       %s listen [TIMEOUT [PORT]]\n\n"
                          + "         Forks the ServerRunnable to listen for the given number\n"
                          + "         of milliseconds (%d by default) on the given port (%d\n"
                          + "         by default.)\n\n"
                          + "         While the server thread is listening, incoming PapasVision\n"
                          + "         XML is fed to the AutoAimCommand in order to simulate turning\n"
                          + "         the robot in response to vision data.\n\n"
                          + "       %s fit ALGORITHM [d v [d v [...]]]\n\n"
                          + "         Calculates a curve of best fit using the given algorithm\n"
                          + "         for the given set of data points.  The algorithm choices\n"
                          + "         right now are:\n\n"
                          + "%s\n"
                          + "         The data point arguments are optional; if supplied, they\n"
                          + "         should be given as a list of d-v pairs, where d is a distance\n"
                          + "         and v is a velocity.  If not provided, semi-random points will\n"
                          + "         be used.\n\n"
                          + "         The output is given as a gnuplot command embedded in a Bash\n"
                          + "         here-doc.  You'll need to obtain gnuplot and run the command\n"
                          + "         using Bash's process substitution feature to generate the graph\n"
                          + "         image:\n\n"
                          + "             source <(java -jar %s [other args])\n\n",
                          programName, programName, DEFAULT_TIMEOUT_MILLISECONDS,
                          ServerRunnable.DEFAULT_PORT, programName,
                          algorithmsList.toString(), programName);
    }
}
