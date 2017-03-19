package org.usfirst.frc.team1759.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.RobotMap;

public class GearDropper extends Subsystem {
		RobotMap robotMap;
		OI oi;
		DoubleSolenoid gear; 
		boolean enabled;
		public GearDropper(DoubleSolenoid gear) {
			this.gear = gear;
		}
	 public void initDefaultCommand() {
		//gear = new DoubleSolenoid(0,1);		//TODO: see what ports to assign the solenoid 
		 										//		to.
		 if(gear != null) {
			 enabled = true;
		 } else {
			 enabled = false;
		 }
	 }
	 public void pushIn() {
		 if(enabled) {
			 gear.set(DoubleSolenoid.Value.kForward);
		 } 
	 } 
	 public void pullOut() {
		 if(enabled) {
			 gear.set(DoubleSolenoid.Value.kReverse);
		 }
	 }
	 public void stop() {
		 if(enabled) {
			 gear.set(DoubleSolenoid.Value.kOff); 
		 }	
	 }
}
