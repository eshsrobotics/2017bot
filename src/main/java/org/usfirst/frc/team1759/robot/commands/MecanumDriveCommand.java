
package org.usfirst.frc.team1759.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team1759.robot.Robot;
import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Robot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import com.ctre.CANTalon;

/**
 *
 */
public class MecanumDriveCommand extends Command {

    public MecanumDriveCommand() {
        // Use requires() here to declare subsystem dependencies
        
        requires(MecanumDriveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        
        Robot.MecanumDriveSubsystem.gyroDrive();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}