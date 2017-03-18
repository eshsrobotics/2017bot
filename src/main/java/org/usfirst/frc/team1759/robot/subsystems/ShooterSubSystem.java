
package org.usfirst.frc.team1759.robot.subsystems;

import com.ctre.CANTalon;

import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.PortAssigner;
import org.usfirst.frc.team1759.robot.RobotMap;

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
	CANTalon shoot_wheel;
	CANTalon feed_wheel;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	shoot_wheel = new CANTalon(PortAssigner.getInstance().getAssignedPort(PortAssigner.SHOOT_WHEEL));
		robotMap = new RobotMap();
		oi = new OI();
    	
    }
    
    public void speedUp() {
    	robotMap.velocity = robotMap.velocity + robotMap.littleAdjust;
    }
    
    public void slowDown() {
    	robotMap.velocity = robotMap.velocity - robotMap.littleAdjust;
    }
    
    public void shoot() {
    	
    }
    
    public void shootManual(double velocity) {
    	if(robotMap.velocity > 1) {
    		robotMap.velocity = 1;
    	}
    	if(robotMap.velocity < 0) {
    		robotMap.velocity = 0;
    	}
    	shoot_wheel.set(velocity);
    	try {
			Thread.sleep(robotMap.shooterTime);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    	shoot_wheel.set(0.0);
    }
}

