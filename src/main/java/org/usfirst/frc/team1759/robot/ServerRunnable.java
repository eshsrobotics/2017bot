package org.usfirst.frc.team1759.robot;

import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.IOException;

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
	 * (This also happens to be the maximum time callers have to wait for {@link run}() to exit after calling 
	 * {@link die}().) 
	 */
	public static final int NETWORK_READ_WAIT_TIME_MILLISECONDS = 5000;
	
	/**
	 * How long we are willing to wait around for the server socket to receive a connection from some client.
	 * 
	 * A value of 3 minutes was chosen because this is about as long as a round in FRC is allowed to run.
	 * if we don't receive a connection by the time this has passed, we missed the round anyway!
	 */
	public static final int NETWORK_CONNECTION_WAIT_TIME_MILLISECONDS = 180000;
	
	/**
	 * The port we actually use (which can be modified in the constructor.)
	 * 
	 * This needs to agree with the roborio_port in config/camera-client.ini in order for the C++
	 * and Java ends to talk to one another.
	 */
	int port;
	
	/**
	 * When set to true, the while-loop inside run() exits from natural causes.
	 */
	private boolean killCameraThread = false;

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
	 * to a {@link PapasData}, and stores it for retrieval by the rest of the robot system.
	 * 
	 * TODO: Where are we going to store the PapasData?
	 */
	@Override
	public void run() {

		System.out.println("***THREAD BEGIN***");
		System.out.println("Waiting for client connection.");

		try (ServerSocket serverSocket = new ServerSocket(port)) {

			XMLParser parser = new XMLParser();

			// Never block for more than this many milliseconds when waiting for
			// a connection.
			serverSocket.setSoTimeout(NETWORK_CONNECTION_WAIT_TIME_MILLISECONDS);

			// Wait for a connection.
			Socket clientSocket = serverSocket.accept();

			// Never block for more than this many milliseconds when reading from the network.
			//
			// This is mostly intended to make readline() calls to clientSocket.getInputStream()
			// return in order to allow us to shut down the thread in a reasonably prompt fashion.
			// It shoudln't affect the actual data we read.
			clientSocket.setSoTimeout(NETWORK_READ_WAIT_TIME_MILLISECONDS);

			String address = getAddressAsString(clientSocket);
			System.out.printf("Connected to %s:%d.  Entering waiting loop.\n", address, clientSocket.getPort());

			while (killCameraThread == false) {
				try {

					// Read the available data one line at a time.
					InputStream inputStream = clientSocket.getInputStream();
					BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
					String s = reader.readLine();

					if (s == null) {
						// Something forcibly disconnected the client socket.  Wait
						// for a reconnection.
						System.err.printf("We seem to be disconnected even though no IOException was thrown.  Waiting for new client connection.\n");
						clientSocket = reconnectToClientSocket(serverSocket, clientSocket);
						address = getAddressAsString(clientSocket);
						System.out.printf("Now connected to %s:%d.  Re-entering waiting loop.\n", address, clientSocket.getPort());
						continue;
					}

					// Print the result (for now.)
					System.out.printf("[debug] Got \"%s\" from the remote connection.\n", s);

					// Parse as a PapasData.  This will throw an exception if the string is
					// not a valid PapasVision XML message.					
					PapasData papasData = parser.parse(s);

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
					// the client..
					System.err.printf("Caught an IO exception: %s.  Waiting for new client connection.\n", e.getMessage());
					clientSocket = reconnectToClientSocket(serverSocket, clientSocket);
					address = getAddressAsString(clientSocket);
					System.out.printf("Now connected to %s:%d.  Re-entering waiting loop.\n", address, clientSocket.getPort());

				} catch (XMLParserException e) {

					// XML parsing errors do not require us to reconnect; however, they
					// are real errors, so we're entitled to air our grievances.
					System.err.printf("[error] Couldn't parse network string as PapasVision XML (\"%s\".)\n  (The XML string was %s)\n", 
							e.getMessage(),
							e.getXmlDocumentString());

				} 
			} // end (while no one has given the signal to kill the thread)

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
			// Something unexpected happened!  Log it, at least. 
			System.err.printf("[error] Caught a fatal exception: %s.\nStack trace:\n", e.getMessage());
			e.printStackTrace(System.err);
		}
		System.out.println("***THREAD END***");
	}

	/**
	 * The method we call to reestablish a client connection whenever things go bad on the network.
	 * 
	 * @param serverSocket A {@link ServerSocket} that is already bound to a port.
	 * @param existingClientSocket An existing client socket created with {@link ServerSocket.accept}().
	 * @return A new socket from a second call to serverSocket.accept(), with appropriately-set timeouts.
	 * @throws IOException
	 * @throws SocketException
	 */
	private Socket reconnectToClientSocket(ServerSocket serverSocket,
			Socket existingClientSocket) throws IOException, SocketException {
		existingClientSocket.close();
		Socket newClientSocket = serverSocket.accept();
		newClientSocket.setSoTimeout(NETWORK_READ_WAIT_TIME_MILLISECONDS);
		return newClientSocket;
	}

	/**
	 * Extracts the IPv4 or IPv6 address from the given socket.
	 * 
	 * @param clientSocket The socket to pluck the address from -- preferably one from 
	 *                     a {@link ServerSocket.accept()} call.
	 * @return A hostname, IPv4 address, or IPv6 address surrounded by brackets.
	 */
	private String getAddressAsString(Socket clientSocket) {
		String address = "";
		byte[] rawAddress = clientSocket.getInetAddress().getAddress();
		if (rawAddress.length == 4) {
			address = clientSocket.getInetAddress().getHostName(); // Likely IPv4.
		} else {
			address = "[" + clientSocket.getInetAddress().getHostName() + "]"; // Likely IPv6.
		}
		return address;
	}

	/**
	 * Put run() out of its misery.
	 */
	public void die() {
		killCameraThread = true;
	}
}