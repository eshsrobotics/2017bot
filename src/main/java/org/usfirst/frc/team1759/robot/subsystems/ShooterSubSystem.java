
package org.usfirst.frc.team1759.robot.subsystems;

import org.usfirst.frc.team1759.robot.LinearRegressionCurveFitter;
import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.PapasData;
import org.usfirst.frc.team1759.robot.RobotMap;
import org.usfirst.frc.team1759.robot.ServerRunnable;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 * @author Uche Akotaobi, Daniel Stamper, Spencer Moore, Aidan Galbreath, and
 *         Ari Berkowicz
 *
 *         This subsystem will be used for shooting, both automatic and manual.
 *
 *         This subsystem manages the motors to the shooting mechanism for the
 *         2017bot. It knows how to fire one ball at a time at the given speed,
 *         and has some smarts to allow it to fir just far enough to hit the
 *         target at the given papasDistance.
 */

public class ShooterSubSystem extends Subsystem {

	private RobotMap robotMap;
	private SpeedController shoot_wheel;
	private SpeedController feed_wheel;
	private ServerRunnable serverRunnable;

	private boolean enabled;

	public ShooterSubSystem(ServerRunnable serverRunnable, SpeedController shoot_wheel, SpeedController feed_wheel) {
		super("Shooter");
		this.serverRunnable = serverRunnable;
		this.shoot_wheel = shoot_wheel;
		this.feed_wheel = feed_wheel;
	}

	/**
	 * Initializes this command.
	 */
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		robotMap = new RobotMap();
		if (shoot_wheel == null) {
			enabled = false;
		} else {
			enabled = true;
		}
	}

	/**
	 * Use the PapasVision data to shoot the ball at the correct velocity to hit
	 * the PapasVision target automatically.
	 */
	public void shoot() {
		if (enabled) {

			PapasData papasData = serverRunnable.getPapasData();

			// We have two curve functions based on the data we gathered on
			// 2017-03-22:
			//
			// v = 0.006d + 0.215 (if we allow for repeated data points)
			// v = 0.005d + 0.272 (if we don't).
			//
			// So, given the papasData distance, we can get the velocity
			// automatically.

			double velocity = 0.006 * papasData.papasDistanceInInches + 0.215;
			shootManual(velocity);
		}
	}

	/**
	 * Shoots the ball at the given velocity for the recommended burst time.
	 * 
	 * @param velocity
	 *            The motor velocity for the shooting wheel, ranging from 0% to
	 *            100% (1.0).
	 */
	public void shootManual(double velocity) {
		shootManual(velocity, RobotMap.feedWheelBurstTimeMilliseconds);
	}

	/**
	 * Fires a volley of balls from the shooter at the given power level for the
	 * given burst time.
	 * 
	 * @param velocity
	 *            The motor velocity for the shooting wheel, ranging from 0% to
	 *            100% (1.0).
	 * @param burstTimeMilliseconds
	 *            THe number of milliseconds to keep the trigger hot.
	 */
	public void shootManual(double velocity, long burstTimeMilliseconds) {
		if (enabled) {
			if (velocity > 1) {
				velocity = 1;
			}
			if (velocity < 0) {
				velocity = 0;
			}

			startShooting(velocity);

			// Wait until the burst time has passed; that's when the volley is
			// done.
			try {
				Thread.sleep(RobotMap.feedWheelBurstTimeMilliseconds);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			stopShooting();
		}
	}

	/**
	 * Activates the shoot wheel, then turns on the feed wheel after the shoot
	 * wheel is hot.
	 * 
	 * @param shootWheelVelocity
	 *            The motor velocity for the shooting wheel, ranging from 0% to
	 *            100% (1.0).
	 */
	public void startShooting(double shootWheelVelocity) {
		if (enabled) {
			shoot_wheel.set(shootWheelVelocity);

			try {
				Thread.sleep(RobotMap.shootWheelRampUpTimeMilliseconds);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			// The feed wheel used to not exist on the test bot. (It does now,
			// but this code allows both scenarios to be supported.)
			if (feed_wheel != null) {
				feed_wheel.set(1.0);
			}

		}
	}
	
	public void updateVelocity(double newShootWheelVelocity) {
		if (enabled && shoot_wheel.get() >= 0.0) {
			shoot_wheel.set(newShootWheelVelocity);
			System.out.println(newShootWheelVelocity);
		}
	}

	/**
	 * Shuts the shooter down without getting a ball stuck in its craw.
	 */
	public void stopShooting() {
		if (enabled) {
			// Cut the feed wheel first.
			feed_wheel.set(0.0);

			// Then cut the shoot wheel after (presumably) the last ball in the
			// queue has been shot out.
			try {
				Thread.sleep(RobotMap.shootWheelRampUpTimeMilliseconds);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			shoot_wheel.set(0.0);
		}
	}
}
