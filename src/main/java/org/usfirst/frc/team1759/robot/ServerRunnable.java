package org.usfirst.frc.team1759.robot;

import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
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
 * This is just a Runnable to host our network-listening code. Better
 * that we idle than the main driver thread.
 */
public class ServerRunnable implements Runnable {
	/**
	 * Any target goal that is greater than this distance will be rejected by
	 * findgoal().
	 */

	/**
	 * The default port we listen on.  
	 */
	public static final int DEFAULT_PORT = 12345;

	/**
	 * The maximum time that we are willing to wait to read something from the network before....waiting
	 * to read something from the network again.
	 * 
	 * (This also happens to be the maximum time callers have to wait for run() to exit after calling die().) 
	 */
	public static final int NETWORK_WAIT_TIME_MILLISECONDS = 5000;
	
	/**
	 * The port we actually use (which can be modified in the constructor.)
	 * 
	 * This needs to agree with the roborio_port in config/camera-client.ini in order for the C++
	 * and Java ends to talk to one another.
	 */
	int port;
	
	/**
	 * When set to true the camerathread will kill itself.
	 */
	public static boolean killCameraThread = false;	

	/**
	 * Creates a new ServerRunnable object listening on the default port.
	 */
	public ServerRunnable() {
		this.port = DEFAULT_PORT;
	}
	
	/**
	 * Creates a new ServerRunnable that listens on the given port.
	 */
	public ServerRunnable(int port) {
		this.port = port;
	}
	
	// This variable gives us one place to tweak the amount of time the camera
	// thread
	// will sleep before taking up CPU again. It also helps for the end of the
	// program:
	// after the main thread sets our death flag above, it only needs to wait
	// for about
	// this many milliseconds for *us* to die before it does itself.

	/**
	 * The thread function that this Runnable executes.
	 * 
	 * It listens on localhost for any incoming connection on our port.  If one is received,
	 * it reads any data sent over that connection until a newline has been sent, converts that
	 * to a PapasData, and stores it for retrieval by the rest of the robot system.
	 * 
	 * TODO: Where are we going to store the PapasData?
	 */
	@Override
	public void run() {
				
		System.out.println("Waiting for client connection.");
		
		try (ServerSocket serverSocket = new ServerSocket(port)) {
			
			XMLParser parser = new XMLParser();
			Socket clientSocket = serverSocket.accept();
			
			// Never block for more than this many milliseconds when waiting for the network.
			//
			// This is mostly intended to make readline() calls to clientSocket.getInputStream()
			// return in a reasonably prompt fashion, but there may be unintended consequences if
			// we catch an IOException for any other reason (since that means we now can now fail
			// to reconnect due to a timeout.)
			//
			// I anticipate live disconnections and reconnections on the robot will be rare, and
			// the initial connection has an unlimited time, so we'll wait and see where this 
			// goes (so to speak.)
			clientSocket.setSoTimeout(NETWORK_WAIT_TIME_MILLISECONDS);
			
			String address = "";
			byte[] rawAddress = clientSocket.getInetAddress().getAddress();
			if (rawAddress.length == 4) {
				address = clientSocket.getInetAddress().getHostName(); // Likely IPv4.
			} else {
				address = "[" + clientSocket.getInetAddress().getHostName() + "]"; // Likely IPv6.
			}
			System.out.printf("Connected to %s:%d.  Entering waiting loop.\n", address, clientSocket.getPort());
			
			while (killCameraThread == false) {
				try {
					
					// Read the available data one line at a time.
					InputStream inputStream = clientSocket.getInputStream();
					BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
					String s = reader.readLine();
	
					// Parse as a PapasData.  This will throw an exception if the string is
					// not a valid PapasVision XML message.					
					PapasData papasData = parser.parse(s);
											
					// Print the result (for now.)
					System.out.printf("[debug] Got \"%s\" from the remote connection.\n", s);
				
					// TODO: In lieu of actually storing the PapasData, we're just going
					// to print it for now.
					System.out.printf("%s\n", papasData);
					
				} catch (SocketTimeoutException e) {
					
					// The readline() timed out.  This isn't an error, and it doesn't 
					// require us to reconnect, so swallow the exception.					
					System.err.printf("[debug] (Still waiting for I/O.)\n");
					
				} catch (IOException e) {
					
					// Perhaps the socket disconnected.  Perhaps the network went down.   Whatever
					// happened, it's not worth worrying about.  Just wait for a reconnection from
					// the client.
					System.err.printf("Caught an IO exception: %s.  Waiting for new client connection.\n", e.getMessage());
					clientSocket.close();
					clientSocket = serverSocket.accept();
					
				} catch (XMLParserException e) {
					
					System.err.printf("[error] Couldn't parse network string as PapasVision XML (\"%s\".)\n  (The XML string was %s)\n", 
							e.getMessage(),
							e.getXmlDocumentString());
					
				} 
			}
			
			// Someone has given the signal to kill the thread.  Close any extant connections.
			try {
				if (!clientSocket.isClosed()) {
					System.out.printf("Closing connection to %s:%d.\n", address, clientSocket.getPort());				
					clientSocket.close();			
				}
			} catch (IOException e) {
				System.err.printf("Caught an IO exception while closing network connection: %s.\n", e.getMessage());
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