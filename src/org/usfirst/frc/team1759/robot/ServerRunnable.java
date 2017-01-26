package org.usfirst.frc.team1759.robot;

/**
 * @author Ryan Lim
 *
 * This is just a Runnable to host our network-listening code.  Better that we idle than the
 * main driver thread.
 */
public class ServerRunnable implements Runnable {
	/**
	 * Any target goal that is greater than this distance will be rejected by
	 * findgoal().
	 */
	

	// When set to true the camerathread will kill itself.
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
		// TODO Auto-generated method stub
		
	
		
		while (killCameraThread == false) {
			
		}

		// As soon as control makes it here, R.I.P.
		System.out.println(Thread.currentThread().getName() + ": I have just been informed that is is my time to die.");
	}
	
	public void die()
	{
		killCameraThread = true;
	}
}