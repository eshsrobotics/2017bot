package org.usfirst.frc.team1759.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;

// Helpful links:
// * WPILib API: http://first.wpi.edu/FRC/roborio/release/docs/java/
// *

/**
 *
 * @author Uche Akotaobi, Daniel Stamper, Spencer Moore, Aidan Galbreath, and Ari Berkowicz
 *
 * This class will be used for shooting, both automatic and manual.
 *
 * This class manages the motors to the shooting mechanism for the 2017bot.  It knows how to fire one ball at a time
 * at the given speed, and has some smarts to allow it to fir just far enough to hit the target at the given
 * papasDistance.
 */
public class Shooter {

        /**
         * The number of milliseconds that the feed wheel's motor needs to be online in order to shoot a single ball.
         */
        public static final long FEED_TIME_MILLISECONDS = 5000;

        /**
         * The number of milliseconds needed for the shoot wheel to be ready to launch the ball.
         */
        public static final long TIME_FOR_SHOOTWHEEL_TO_ACCELERATE_MILLISECONDS = 500;

        /**
         * Our feeding wheel controller.
         *
         * Doesn't matter if it's a CANTalon or a Spark, as long as it has set().
         */
        private SpeedController feedWheel = null;

        /**
         * Our shooting wheel controller.
         *
         * Doesn't matter if it's a CANTalon or a Spark, as long as it has set().
         */
        private SpeedController shootWheel = null;

        /**
         * Added to use recent input from PapasData from the camera.
         */
        private PapasData lastReceivedPapasData = null;

        /**
         * Constructs Shooter object given the Talons assigned to each shooter motor.
         *
         * We expect the controllers passed into this constructor to already be initialized with a port,
         * and we only plan to test with Sparks and CANTalons (both of which implement the {@link SpeedController}
         * interface.)
         *
         * @param feedWheel The wheel which, when activated, feeds a ball into the active shooting mechanism.
         *                  It should already have been initialized and assigned to a port.
         * @param shootWheel The wheel which, when activated, will launch a fed ball out into the ether.  It
         *                   should already have been initialized and assigned to a port as well.
         */
        public Shooter(SpeedController feedWheel, SpeedController shootWheel) {
                this.feedWheel = feedWheel;
                this.shootWheel = shootWheel;
        }

        /**
         * Stores the last camera data
         * @param recievedPapasData used to input the last PapasData received from the camera.
         */
        public void receivedPapasData(PapasData recievedPapasData) {
                lastReceivedPapasData = recievedPapasData;
        }

        /**
         * Fires a ball.  Uses driver input to decide velocity, as opposed to deriving velocity from PapasData.
         *
         * @param velocity from 0 to 1 to determine shooting speed.
         *
         */
        public void fire(double velocity) {
                // Initial state: Both wheels are off. Assuming that we are in percentvbus mode so values are between -1 and 1
                if(velocity < 0) {
                        velocity = 0;
                }
                if(velocity > 1) {
                        velocity = 1;
                }
                try {

                        // Ramp up the shooting wheel.  This takes finite time to do.
                        shootWheel.set(velocity); //Starting shootWheel first to avoid inaccuracies due to motor acceleration
                        Thread.sleep(TIME_FOR_SHOOTWHEEL_TO_ACCELERATE_MILLISECONDS);

                        // While the shooting wheel is hot, load a ball (hopefully just one.)
                        feedWheel.set(1.0);
                        Thread.sleep(FEED_TIME_MILLISECONDS);

                        // Done firing.
                        feedWheel.set(0.0);
                        shootWheel.set(0.0);

                } catch(Exception e) {
                        System.err.printf("Caught an exception: %s \n", e.getMessage());
                }
        }


        /**
         * Fires a ball.  Uses the last {@link receivedPapasData} to determine the velocity by means of our curve-fitting function.
         *
         * Calls {@link fire(double)} once the velocity has been determined.
         *
         * TODO: Implement this.
         */
        public void fire() {
        }

        /**
         * A function that uses a polynomial curve-fitting function to interpolate the ideal velocity to fire the ball
         * at, given our (camera's) distance from the target('s reflective tape.)
         *
         * TODO: Gather the data to derive a curve for this.
         *
         * @param distanceInInches Nominally {@link this.lastReceivedPapasData.papasDistance}, though you can pass any number
         *                         you like in here.  If the distance is less than or equal to 0, we'll return 0 and make fun
         *                         of you in the logs.
         * @return A velocity value to feed to the firing wheel.  The velocity will always be in the closed interval [0, 1],
         *         where 0 is minimum velocity and 1 is maximum velocity.
         */
        private double convertPapasDistanceToVelocity(double distanceInInches) {
                return 0;
        }
}
