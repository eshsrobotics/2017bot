package org.usfirst.frc.team1759.robot.subsystems;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.PortAssigner;
import org.usfirst.frc.team1759.robot.RobotMap;

public class GearDropper extends Subsystem {
		RobotMap robotMap;
		OI oi;
		PortAssigner portAssigner;
		DoubleSolenoid gear; 
		CANTalon release;
	 public void initDefaultCommand() {
		gear = new DoubleSolenoid(0,1);		//TODO: see what ports to assign the solenoid 
											//		to.
		release = new CANTalon(8); 
	 }
	 public void pushIn() {
		 gear.set(DoubleSolenoid.Value.kForward);
	 }
	 public void pullOut() {
		 gear.set(DoubleSolenoid.Value.kReverse);
	 }
	 public void stop() {
	 	gear.set(DoubleSolenoid.Value.kOff);
	 }
	 public void release() {
		 release.set(.5);
	 }
}
