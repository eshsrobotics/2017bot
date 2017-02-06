/**
 * 
 */
package org.usfirst.frc.team1759.terminal;

import org.usfirst.frc.team1759.robot.XMLParser;
import org.usfirst.frc.team1759.robot.PapasData;

/**
 * @author uakotaobi
 *
 */
public class TerminalTest {

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
			} else {
				System.out.println(String.format("[debug] unrecognized sub-command \"%s\" for argument 1", subCommand));
			}
		}
	}

	/**
	 * Prints a usage message.
	 */
	public static void usage() {
		String programName = System.getProperty("sun.java.command");

		System.out.println(String.format(
				"Usage: %s parse XML-STRING\n\n" + "         Attempts to parse the given PapasVision XML string.\n\n"
						+ "       %s listen PORT TIMEOUT\n\n"
						+ "         Forks the ServerRunnable to listen for the given number\n"
						+ "         of milliseconds on the given port.\n",
				programName, programName));
	}
}
