
package org.usfirst.frc.team1759.robot;

import org.usfirst.frc.team1759.robot.subsystems.MecanumDriveSubSystem;
import org.usfirst.frc.team1759.robot.subsystems.GearDropper;
import org.usfirst.frc.team1759.robot.subsystems.ShooterSubSystem;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
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

	private ServerRunnable runnable = new ServerRunnable(12345); // Used to
																	// receive
																	// information
																	// from
																	// PapasData,
																	// or the
																	// lies we
																	// feed it.
	private Thread papasThread = null; // Thread that runs our ServerRunnable
	// public static final ExampleSubsystem exampleSubsystem = new
	// ExampleSubsystem();
	OI oi;

	Command autonomousCommand;
	SendableChooser<?> chooser;
	String autoSelected;

	RobotDrive myRobot;
	Joystick leftStick;
	Joystick rightStick;
	Joystick shootStick;
	PortAssigner portAssigner;
	CANTalon back_right_wheel;
	CANTalon front_right_wheel;
	CANTalon back_left_wheel;
	CANTalon front_left_wheel;
	CANTalon shoot_wheel;
	CANTalon feed_wheel;
	CANTalon gear_deliver;
	CameraServer server;
	XMLParser xmlParser;
	PapasData papasData;
	ServerRunnable serverRunnable;
	DoubleSolenoid gearSolenoid;
	MecanumDriveSubSystem papasDrive;
	ShooterSubSystem shooter;
	GearDropper gear;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		oi = new OI(); // TODO: OI.java see if neccessary.
		// Define Autonomous mode options and display on Driver station dash.
		chooser = new SendableChooser<Object>();
		SmartDashboard.putData("Auto choices", chooser);

		xmlParser = new XMLParser();
		papasData = new PapasData();
		serverRunnable = new ServerRunnable();
		papasDrive = new MecanumDriveSubSystem();
		shooter = new ShooterSubSystem();
		gear = new GearDropper();
		server = CameraServer.getInstance();
		server.startAutomaticCapture();
		// Initalize talons.
		CANTalon talons[] = new CANTalon[10];
		for (int i = 0; i < talons.length; ++i) {
			talons[i] = new CANTalon(i);
		}

		gearSolenoid = new DoubleSolenoid(0, 1);

		Sensors.gyro.reset();
		Sensors.gyro.calibrate();

		papasThread = new Thread(runnable);
		papasThread.setName("PapasData reception");
		papasThread.start();

		/*
		 * If you draw an imaginary "И" (Cyrillic ee) on the top of the robot
		 * starting from the front left wheel, the "И" will end with the back
		 * right wheel and will hit the talons in numerical order.
		 */
		front_left_wheel = talons[0];
		back_left_wheel = talons[1];
		front_right_wheel = talons[2];
		back_right_wheel = talons[3];
		shoot_wheel = talons[4];
		feed_wheel = talons[5];
		gear_deliver = talons[9];

		// Inverting signal since they are wired in reverse polarity on the
		// robot
		talons[0].setInverted(true);
		talons[1].setInverted(true);
		talons[2].setInverted(false);
		talons[3].setInverted(false);

		/*
		 * load talon port (cantalon), lower shoot talon port(cantalon), upper
		 * shoot talon port(cantalon)
		 */
		leftStick = new Joystick(0);
		rightStick = new Joystick(1);
		shootStick = new Joystick(2);

		Sensors.rightBack.setMaxPeriod(.1);
		Sensors.rightBack.setMinRate(10);
		Sensors.rightBack.setDistancePerPulse(5);
		Sensors.rightBack.setReverseDirection(false);
		Sensors.rightBack.setSamplesToAverage(7);
		Sensors.rightFront.setMaxPeriod(.1);
		Sensors.rightFront.setMinRate(10);
		Sensors.rightFront.setDistancePerPulse(5);
		Sensors.rightFront.setReverseDirection(false);
		Sensors.rightFront.setSamplesToAverage(7);
		Sensors.leftBack.setMaxPeriod(.1);
		Sensors.leftBack.setMinRate(10);
		Sensors.leftBack.setDistancePerPulse(5);
		Sensors.leftBack.setReverseDirection(false);
		Sensors.leftBack.setSamplesToAverage(7);
		RobotMap.accX = Sensors.accel.getX();
		RobotMap.accY = Sensors.accel.getY();
		RobotMap.accZ = Sensors.accel.getZ();
		RobotMap.accTotal = Math.sqrt((RobotMap.accX * RobotMap.accX) + (RobotMap.accZ * RobotMap.accZ));

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
		front_left_wheel.set(0);
		front_right_wheel.set(0);
		back_left_wheel.set(0);
		back_right_wheel.set(0);
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
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
			if (rightStick.getRawButton(12) == true) {
				RobotMap.gyroIO = !RobotMap.gyroIO; // Tells the code to start
													// using the gyro or
													// to stop using the gyro,
													// depending on the
													// state of the variable.
			}
			
			if (RobotMap.gyroIO == false) {
				papasDrive.manualDrive();
			} else {
				papasDrive.gyroDrive();
			}
			// Firing mechanism.
			if (leftStick.getRawButton(3)) {
				shooter.slowDown();
			}
			if (leftStick.getRawButton(4)) {
				shooter.speedUp();
			}
			if (leftStick.getTrigger()) {
				shooter.shootManual(RobotMap.testShooterSpeed);
			}
					
			if(leftStick.getRawButton(9)) {
				gear.pullOut();
			} else if(leftStick.getRawButton(10)) {
				gear.pushIn();
			} else {
				gear.stop();
			}


			/**
			* Used for testing speed on the wheels.
			*/

			System.out.println("Speed of front right motor: " + Sensors.rightFront.getRate());
			System.out.println("Speed of front left motor: " + Sensors.leftFront.getRate());
			System.out.println("Speed of back right motor: " + Sensors.rightBack.getRate());
			System.out.println("Speed of back left motor: " + Sensors.leftBack.getRate());

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
		runnable.die();
		try {
			papasThread.join();
		} catch (Throwable t) {
			// Swallow the exception, but log first.
			System.err.println(t.getMessage());
		}
	}
}
