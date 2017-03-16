package org.usfirst.frc.team1759.robot.subsystems;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team1759.robot.OI;
import org.usfirst.frc.team1759.robot.PortAssigner;
import org.usfirst.frc.team1759.robot.RobotMap;

public class GearDropper extends Subsystem {
	 public void initDefaultCommand() {
		 RobotMap robotMap;
		 OI oi;
		 PortAssigner portAssigner;
		 DoubleSolenoid gearSolenoid; 
		 CANTalon release;

		gearSolenoid = new DoubleSolenoid(0,1);		//TODO: see what ports to assign the solenoid 
													//		to.
		release = new CANTalon(8); 
		 
	 }
	 public void pushIn() {
		 set(gearSolenoid.Value.kForward);
	 }
	 public void pullOut() {
		 set(gearSolenoid.Value.kReverse);
	 }
	 public void stop() {
	 	set(gearSolenoid.Value.kOff);
	 }
	 public void release() {
		 release.set(.5);
	 }
}
