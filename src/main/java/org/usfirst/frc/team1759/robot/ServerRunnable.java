package org.usfirst.frc.team1759.robot;

import java.net.ServerSocket;
import java.net.Socket;
import java.io.PrintWriter;
import java.io.BufferedReader;
import java.io.File;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetAddress;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

/**
 * @author Ari Berkowicz and Ryan Lim
 *
 *         This is just a Runnable to host our network-listening code. Better
 *         that we idle than the main driver thread.
 */
public class ServerRunnable implements Runnable {
	/**
	 * Any target goal that is greater than this distance will be rejected by
	 * findgoal().
	 */

	// When set to true the camerathread will kill itself.
	public static final int PORT = 12345;
	public static boolean killCameraThread = false;	

	// This variable gives us one place to tweak the amount of time the camera
	// thread
	// will sleep before taking up CPU again. It also helps for the end of the
	// program:
	// after the main thread sets our death flag above, it only needs to wait
	// for about
	// this many milliseconds for *us* to die before it does itself.

	@Override
	public void run() {
		Socket clientSocket = null;
		
		try (ServerSocket socket = new ServerSocket(PORT)) {

			System.out.println("Waiting for client connection.");
			clientSocket = socket.accept();
			
			System.out.println("Entering waiting loop.");
			while (killCameraThread == false) {
				try {
									
					InputStream inputStream = clientSocket.getInputStream();
					BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
					String s = reader.readLine();
	
					String address = "";
					byte[] rawAddress = clientSocket.getInetAddress().getAddress();
					if (rawAddress.length == 4) {
						address = clientSocket.getInetAddress().getHostName(); // Likely
																				// IPv4.
					} else {
						address = "[" + clientSocket.getInetAddress().getHostName() + "]"; // Likely
																							// IPv6.
					}
					System.out.println("Got \"" + s + "\" from " + address + ":" + clientSocket.getPort());
				
				} catch (IOException e) {
					
					// Perhaps the socket disconnected.  Perhaps the network went down.   Whatever
					// happened, it's not worth worrying about.  Just wait for a reconnection from
					// the client.
					System.err.println("Caught an IO exception: " + e.getMessage() + ".  Waiting for new client connection.");
					clientSocket.close();
					clientSocket = socket.accept();
				} 
			}
					
		} catch (Throwable e) {
			// Gets rid of all the errors from before.
			System.err.println("Caught a fatal exception: " + e.getMessage());
		}
	}

	public void die() {
		killCameraThread = true;
	}
}