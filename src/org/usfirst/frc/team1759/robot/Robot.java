
package org.usfirst.frc.team1759.robot;

import org.usfirst.frc.team1759.robot.commands.AutonomousPhaseCommand;
import org.usfirst.frc.team1759.robot.commands.ManualFireCommand;
import org.usfirst.frc.team1759.robot.subsystems.BallIntakeSubSystem;
import org.usfirst.frc.team1759.robot.subsystems.MecanumDriveSubSystem;
//import org.usfirst.frc.team1759.robot.subsystems.GearDropperSubSystem;
import org.usfirst.frc.team1759.robot.subsystems.ShooterSubSystem;

import com.ctre.CANTalon;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
//import edu.wpi.first.wpilibj.command.Command;
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

	AutonomousPhaseCommand autoPhaseCommand;
	ManualFireCommand manualFireCommand;
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
	// GearDropperSubSystem gear;
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
		papasDrive = new MecanumDriveSubSystem(new CANTalon(RobotMap.back_right_wheel),
				new CANTalon(RobotMap.front_right_wheel), new CANTalon(RobotMap.back_left_wheel),
				new CANTalon(RobotMap.front_left_wheel));
		ballGrabber = new BallIntakeSubSystem(new CANTalon(RobotMap.ballIntake));
		// gear = new GearDropperSubSystem(null);
		// gear = new GearDropperSubSystem(new
		// DoubleSolenoid(RobotMap.gearSolenoid1, RobotMap.gearSolenoid2));
		shooting = new ShooterSubSystem(serverRunnable, new CANTalon(RobotMap.shoot_wheel),
				new CANTalon(RobotMap.feed_wheel));
		autoPhaseCommand = new AutonomousPhaseCommand(papasDrive, 1); //
		// Change
		// the
		// integer
		// to
		// change
		// the
		// mode,
		// because
		// I
		// didn't
		// want
		// to
		// mess
		// with
		// the
		// sendableChooser.
		// 1 =
		// drive
		// forward,
		// 2 =
		// right,
		// 3 is
		// left.

		// Not Needed Anymore
		// server = CameraServer.getInstance();
		// server.startAutomaticCapture();
		//
		// papasThread = new Thread(serverRunnable);
		// papasThread.setName("PapasData reception");
		// papasThread.start();

		RobotMap.accX = Sensors.accel.getX();
		RobotMap.accY = Sensors.accel.getY();
		RobotMap.accZ = Sensors.accel.getZ();
		RobotMap.accTotal = Math.sqrt((RobotMap.accX * RobotMap.accX) + (RobotMap.accZ * RobotMap.accZ));

		speed = .05;
		oi = new OI(papasDrive, serverRunnable);
		manualFireCommand = new ManualFireCommand(shooting, oi.shootStick);

		// Camera Streamer | TEAM 4
		new Thread(() -> {

			// Sets up USB cam 0 is id
			UsbCamera cam1 = CameraServer.getInstance().startAutomaticCapture(0);

			// Set FPS
			cam1.setFPS(15);
			// Changes Resolution (width, height)
			cam1.setResolution(160, 120);
			// Sets up USB cam1 is id
			UsbCamera cam2 = CameraServer.getInstance().startAutomaticCapture(1);

			// Set FPS
			cam2.setFPS(15);
			// Changes Resolution (width, height)
			cam2.setResolution(160, 120);

		}).start();
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

		// autoSelected = (String) chooser.getSelected();
		// System.out.println("Auto selected: " + autoSelected);
		autoPhaseCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		// switch (autoSelected) {
		// case customAuto:
		// // Put custom auto code here
		// // autoPhaseCommand.execute();
		// break;
		// case defaultAuto:
		//
		// default:
		// // Put default auto code here
		// break;
		// }
		// papasDrive.autonomousDrive(0.25, 0.25, 0.25);
		Scheduler.getInstance().run();
	}

	public void teleopInit() {

		/*
		 * This makes sure that the autonomous stops running when teleop starts
		 * running. If you want the autonomous to continue until interrupted by
		 * another command, remove this line or comment it out.
		 */
		if (autoPhaseCommand != null) {
			autoPhaseCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

		Scheduler.getInstance().run();
		// notice we are clamping minimum values
		oi.limitThreshold();
		// Drive

		papasDrive.manualDrive(oi.thresholdedX, oi.thresholdedY, oi.thresholdedTwist);

		// Manual shooting.
		if (oi.shootStick.getTrigger() && !manualFireCommand.isRunning()) {

			// As soon as the user releases the trigger, isRunning() will become
			// false
			// and manualFireCommand.end() will be called automatically, turning
			// off
			// the feed and shooting wheels (in that order.)
			manualFireCommand.start();
		}
		if (manualFireCommand.isRunning()) {
			shooting.updateVelocity(manualFireCommand.getVelocityFromJoystick());
		}

		// Gear Delivery

		// if (oi.gearIn != null) {
		// gear.pushIn();
		// } else if (oi.gearOut != null) {
		// gear.pullOut();
		// } else {
		// gear.stop();
		// }

		// Ball Intake
		if (oi.ballOut != null) {
			ballGrabber.BallIn();
		} else if (oi.ballIn != null) {
			ballGrabber.BallOut();
		} else {
			ballGrabber.stop();
		}

		/**
		 * Used for testing speed on the wheels.
		 */

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
