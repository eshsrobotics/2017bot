
package org.usfirst.frc.team1759.robot.subsystems;

import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.RobotMap;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
*
* @author Uche Akotaobi, Daniel Stamper, Spencer Moore, Aidan Galbreath, and Ari Berkowicz
*
* This subsystem will be used for shooting, both automatic and manual.
*
* This subsystem manages the motors to the shooting mechanism for the 2017bot.  It knows how to fire one ball at a time
* at the given speed, and has some smarts to allow it to fir just far enough to hit the target at the given
* papasDistance.
*/
public class ShooterSubSystem extends Subsystem {
	RobotMap robotMap;
	OI oi;
	SpeedController shoot_wheel;
	SpeedController feed_wheel;
	boolean enabled;
	public ShooterSubSystem(SpeedController shoot_wheel, SpeedController feed_wheel) {
		this.shoot_wheel = shoot_wheel;
		this.feed_wheel = feed_wheel;
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
		robotMap = new RobotMap();
		if(shoot_wheel == null){
			enabled = false;
		} else {
			enabled = true;
		}
    }
    
    public void speedUp() {
    		RobotMap.velocity = RobotMap.velocity + RobotMap.littleAdjust;
    }
    
    public void slowDown() {
    		RobotMap.velocity = RobotMap.velocity - RobotMap.littleAdjust;
    }
    
    /**
     * For automatic shooting, with input from PapasVision.
     */
    public void shoot() {
    	if(enabled) {
    		
    	}
    }
    
    public void shootManual(double velocity) {
    	if(enabled) {
    		if(RobotMap.velocity > 1) {
    			RobotMap.velocity = 1;
    		}
    		if(RobotMap.velocity < 0) {
    			RobotMap.velocity = 0;
    		}
    		shoot_wheel.set(velocity);
    		try {
    			Thread.sleep(RobotMap.shooterTime);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}   
    		
    		// On the test bot, until feed wheel exists, it's passed as null.
    		// Still want to shoot, so the subsystem shouldn't rely on that. If it's null, don't use it.
    		if(feed_wheel != null){
    			feed_wheel.set(1);
    			try {
					Thread.sleep(RobotMap.feedTime);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
    		}
    	}
    }
    public void stop() {
    	shoot_wheel.set(0.0);
		feed_wheel.set(0.0);
    }
}

