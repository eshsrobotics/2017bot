/**
 * 
 */
package org.usfirst.frc.team1759.terminal;

import org.usfirst.frc.team1759.robot.ServerRunnable;
import org.usfirst.frc.team1759.robot.XMLParser;
import org.usfirst.frc.team1759.robot.PapasData;

/**
 * @author uakotaobi
 *
 */
public class TerminalTest {

	final static int DEFAULT_TIMEOUT_MILLISECONDS = 15000;
	
	/**
	 * @param args The command-line arguments.  args[0] must be either "parse" or "listen" (for now.)  
	 */
	public static void main(String[] args) throws Exception{

		if (args.length == 0) {
			
			System.err.println("[error] Missing sub-command; must be either 'parse' or 'listen'.");
			usage();
			
		} else {
			
			// Sanity check: print our args.
			System.out.print("[debug] Program arguments: ");
			for (int i = 0; i < args.length; i++) {
				System.out.print(String.format("\"%s\" ", args[i]));
			}
			System.out.print("\n");

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

		System.out.printf(
				"Usage: %s parse XML-STRING\n\n" + "         Attempts to parse the given PapasVision XML string.\n\n"
						+ "       %s listen [TIMEOUT [PORT]]\n\n"
						+ "         Forks the ServerRunnable to listen for the given number\n"
						+ "         of milliseconds (%d by default) on the given port (%d\n"
						+ "         by default.)\n",
				programName, programName, DEFAULT_TIMEOUT_MILLISECONDS, ServerRunnable.DEFAULT_PORT);
	}
}
