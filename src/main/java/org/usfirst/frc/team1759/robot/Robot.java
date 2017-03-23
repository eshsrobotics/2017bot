
package org.usfirst.frc.team1759.robot;

import org.usfirst.frc.team1759.robot.subsystems.MecanumDriveSubSystem;
import org.usfirst.frc.team1759.robot.subsystems.BallIntakeSubSystem;
import org.usfirst.frc.team1759.robot.subsystems.GearDropperSubSystem;
import org.usfirst.frc.team1759.robot.subsystems.ShooterSubSystem;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	private Thread papasThread = null; // Thread that runs our ServerRunnable
	// public static final ExampleSubsystem exampleSubsystem = new
	// ExampleSubsystem();

	Command autonomousCommand;
	SendableChooser<?> chooser;
	String autoSelected;

	RobotDrive myRobot;

	Joystick rightStick;
	CameraServer server;
	XMLParser xmlParser;
	PapasData papasData;
	ServerRunnable serverRunnable;
	MecanumDriveSubSystem papasDrive;
	BallIntakeSubSystem ballGrabber;
	GearDropperSubSystem gear;
	ShooterSubSystem shooting;
	double speed;

	OI oi;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {

		// Define Autonomous mode options and display on Driver station
		// dash.
		chooser = new SendableChooser<Object>();
		SmartDashboard.putData("Auto choices", chooser);

		xmlParser = new XMLParser();
		papasData = new PapasData();
		serverRunnable = new ServerRunnable();
		papasDrive = new MecanumDriveSubSystem(new Jaguar(RobotMap.back_right_wheel),
				new Jaguar(RobotMap.front_right_wheel), new Jaguar(RobotMap.back_left_wheel),
				new Jaguar(RobotMap.front_left_wheel));
		ballGrabber = new BallIntakeSubSystem(new Jaguar(RobotMap.feeder));
		gear = new GearDropperSubSystem(null);
		// gear = new GearDropperSubSystem(new
		// DoubleSolenoid(RobotMap.gearSolenoid1, RobotMap.gearSolenoid2));
		shooting = new ShooterSubSystem(new Jaguar(RobotMap.shoot_wheel), new Jaguar(RobotMap.feed_wheel));

		server = CameraServer.getInstance();
		server.startAutomaticCapture();

		papasThread = new Thread(serverRunnable);
		papasThread.setName("PapasData reception");
		papasThread.start();

		RobotMap.accX = Sensors.accel.getX();
		RobotMap.accY = Sensors.accel.getY();
		RobotMap.accZ = Sensors.accel.getZ();
		RobotMap.accTotal = Math.sqrt((RobotMap.accX * RobotMap.accX) + (RobotMap.accZ * RobotMap.accZ));

		speed = .05;
		oi = new OI(papasDrive, serverRunnable);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	public void disabledInit() {

	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	public void autonomousInit() {

		autoSelected = (String) chooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
		autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:

		default:
			// Put default auto code here
			break;
		}

		Scheduler.getInstance().run();
	}

	public void teleopInit() {

		/*
		 * This makes sure that the autonomous stops running when teleop starts
		 * running. If you want the autonomous to continue until interrupted by
		 * another command, remove this line or comment it out.
		 */
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		// notice we are clamping minimum values
		oi.limitThreshold();

		if (oi.driveSwitch != null) {
			RobotMap.gyroIO = !RobotMap.gyroIO;
		}
		
		// Drive
		
		if(RobotMap.gyroIO) {
			papasDrive.gyroDrive(oi.thresholdedX, oi.thresholdedY, oi.thresholdedTwist);
		} else {	
			papasDrive.manualDrive(oi.thresholdedX, oi.thresholdedY, oi.thresholdedTwist);
		}
		
		// Manual Shooting
		
		if (rightStick.getTrigger()) {
			shooting.shootManual(RobotMap.velocity);
		} else if (oi.goSlow != null) {
			shooting.slowDown();
		} else if (oi.goFast != null) {
			shooting.speedUp();
		} else {
			shooting.stop();
		}
		
		// Gear Delivery
		
		if(oi.gearIn != null) {
			gear.pushIn();
		} else if(oi.gearOut != null) {
			gear.pullOut();
		} else {
			gear.stop();
		}
		
		//Ball Intake
		
		if(oi.ballIn != null) {
			ballGrabber.BallIn();
		} else if (oi.ballOut != null) {
			ballGrabber.BallOut();
		} else {
			ballGrabber.stop();
		}

		/**
		 * Used for testing speed on the wheels.
		 */
		Scheduler.getInstance().run();
		}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}

	/**
	 * This function is called when the thread dies.
	 */
	public void finalize() {
		serverRunnable.die();
		try {
			papasThread.join();
		} catch (Throwable t) {
			// Swallow the exception, but log first.
			System.err.println(t.getMessage());
		}
	}
}
