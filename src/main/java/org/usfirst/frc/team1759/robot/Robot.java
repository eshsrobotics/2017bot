
package org.usfirst.frc.team1759.robot;

import org.usfirst.frc.team1759.robot.ServerRunnable;
import org.usfirst.frc.team1759.robot.XMLParser;
import org.usfirst.frc.team1759.robot.PapasData;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import java.lang.Math;

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
	SendableChooser chooser;
	String autoSelected;

	RobotDrive myRobot;
	Joystick leftStick;
	Joystick rightStick;
	Joystick shootStick;
	CANTalon back_right_wheel;
	CANTalon front_right_wheel;
	CANTalon back_left_wheel;
	CANTalon front_left_wheel;
	CANTalon shoot_wheel;
	CANTalon feed_wheel;
	CANTalon gear_deliver;
	CANTalon gear_tilt;
	Encoder rightBack;
	Encoder rightFront;
	Encoder leftBack;
	Encoder leftFront;
	Encoder shootWheel;
	ADXRS450_Gyro gyro;
	BuiltInAccelerometer accel;
	Shooter shooter;
	CameraServer server;
	XMLParser xmlParser;
	PapasData papasData;
	ServerRunnable serverRunnable;
	DoubleSolenoid gearSolenoid;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		oi = new OI(); // TODO: OI.java see if neccessary.
		// Define Autonomous mode options and display on Driver station dash.
		chooser = new SendableChooser();
		SmartDashboard.putData("Auto choices", chooser);

		xmlParser = new XMLParser();
		papasData = new PapasData();
		serverRunnable = new ServerRunnable();

		server = CameraServer.getInstance();
		server.startAutomaticCapture();
		// Initalize talons.
		CANTalon talons[] = new CANTalon[10];
		for (int i = 0; i < talons.length; ++i) {
			talons[i] = new CANTalon(i);
		}

		gearSolenoid = new DoubleSolenoid(0, 1);

		gyro = new ADXRS450_Gyro();
		gyro.reset();
		gyro.calibrate();

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
		gear_tilt = talons[8];

		shooter = new Shooter(shoot_wheel, feed_wheel);

		// Inverting signal since they are wired in reverse polarity on the
		// robot
		talons[0].setInverted(true);
		talons[1].setInverted(true);
		talons[2].setInverted(false);
		talons[3].setInverted(false);

		// front left, back left, front right, back right
		myRobot = new RobotDrive(front_left_wheel, back_left_wheel, front_right_wheel, back_right_wheel);

		/*
		 * load talon port (cantalon), lower shoot talon port(cantalon), upper
		 * shoot talon port(cantalon)
		 */
		leftStick = new Joystick(0);
		rightStick = new Joystick(1);
		shootStick = new Joystick(2);

		Encoder rightBack = new Encoder(6, 7, false, CounterBase.EncodingType.k2X);
		rightBack.setMaxPeriod(.1);
		rightBack.setMinRate(10);
		rightBack.setDistancePerPulse(5);
		rightBack.setReverseDirection(false);
		rightBack.setSamplesToAverage(7);
		Encoder rightFront = new Encoder(4, 5, false, CounterBase.EncodingType.k2X);
		rightFront.setMaxPeriod(.1);
		rightFront.setMinRate(10);
		rightFront.setDistancePerPulse(5);
		rightFront.setReverseDirection(false);
		rightFront.setSamplesToAverage(7);
		Encoder leftBack = new Encoder(2, 3, false, CounterBase.EncodingType.k2X);
		leftBack.setMaxPeriod(.1);
		leftBack.setMinRate(10);
		leftBack.setDistancePerPulse(5);
		leftBack.setReverseDirection(false);
		leftBack.setSamplesToAverage(7);
		Encoder leftFront = new Encoder(0, 1, false, CounterBase.EncodingType.k2X);
		// Encoder shoot = new Encoder(8, 9, false,
		// CounterBase.EncodingType.k2X);
		// shoot.setMaxPeriod(.1);
		// shoot.setMinRate(10);
		// shoot.setDistancePerPulse(5);
		// shoot.setReverseDirection(false);
		// shoot.setSamplesToAverage(7);
		accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);
		RobotMap.accX = accel.getX();
		RobotMap.accY = accel.getY();
		RobotMap.accZ = accel.getZ();
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
		try {
			// TODO: Enter gyro angle reading into last parameter.
			double Kp = 0.03;
			double angle = gyro.getAngle();
			double rightStickX = 0;
			double rightStickY = 0;
			double rightStickTwist = 0;
			double accStart = 0;
			myRobot.setMaxOutput(RobotMap.medium);
			if (Math.abs(rightStick.getX()) > RobotMap.thresholdX) {
				rightStickX = rightStick.getX();
			}
			if (Math.abs(rightStick.getY()) > RobotMap.thresholdY) {
				rightStickY = rightStick.getY();
			}
			if (Math.abs(rightStick.getTwist()) > RobotMap.thresholdTwist) {
				rightStickTwist = rightStick.getTwist();
			}
			if (rightStick.getRawButton(12) == true) {
				RobotMap.gyroIO = !RobotMap.gyroIO; // Tells the code to start using the gyro or
									// to stop using the gyro, depending on the
									// state of the variable.
			}
			if (RobotMap.gyroIO == false) {
				myRobot.mecanumDrive_Cartesian(-rightStickX, -rightStickY, -rightStickTwist, 0);
			} else {
				myRobot.mecanumDrive_Cartesian(-rightStickX, -rightStickY, -rightStickTwist, angle * Kp);
			}
			if (leftStick.getRawButton(11)) {
				gearSolenoid.set(DoubleSolenoid.Value.kForward);
			} else if (leftStick.getRawButton(12)) {
				gearSolenoid.set(DoubleSolenoid.Value.kReverse);
			} else {
				gearSolenoid.set(DoubleSolenoid.Value.kOff);
			}


			if (rightStickX == 0 && rightStickY == 0 && rightStickTwist == 0) {
				if (RobotMap.accTotal != 0) {
					front_right_wheel.set(RobotMap.littleAdjust);
					back_right_wheel.set(RobotMap.littleAdjust);
					front_left_wheel.set(-RobotMap.littleAdjust);
					back_left_wheel.set(-RobotMap.littleAdjust);
					if (RobotMap.accTotal == 0) {
						myRobot.setMaxOutput(RobotMap.medium);

						if (Math.abs(rightStick.getX()) > RobotMap.thresholdX) {
							rightStickX = rightStick.getX();
						}
						if (Math.abs(rightStick.getY()) > RobotMap.thresholdY) {
							rightStickY = rightStick.getY();
						}
						if (Math.abs(rightStick.getTwist()) > RobotMap.thresholdTwist) {
							rightStickTwist = rightStick.getTwist();
						}
						if (rightStick.getRawButton(5)) {
							front_right_wheel.set(RobotMap.max);
							front_left_wheel.set(RobotMap.max);
							back_right_wheel.set(RobotMap.max);
							back_left_wheel.set(RobotMap.max);
						}
						if (rightStick.getRawButton(3)) {
							front_right_wheel.set(RobotMap.high);
							front_left_wheel.set(RobotMap.high);
							back_right_wheel.set(RobotMap.high);
							back_left_wheel.set(RobotMap.high);
						}
						if (rightStick.getRawButton(4)) {
							front_right_wheel.set(RobotMap.low);
							front_left_wheel.set(RobotMap.low);
							back_right_wheel.set(RobotMap.low);
							back_left_wheel.set(RobotMap.low);
						}
						myRobot.mecanumDrive_Cartesian(rightStickY, -rightStickX, -rightStickTwist, angle * Kp);
						// myRobot.mecanumDrive_Cartesian(rightStick.getY(),
						// rightStick.getX(), rightStick.getTwist(), 0);

						if (rightStickX == 0 && rightStickY == 0 && rightStickTwist == 0) {
							if (RobotMap.accTotal != 0) {
								front_right_wheel.set(RobotMap.littleAdjust);
								back_right_wheel.set(RobotMap.littleAdjust);
								front_left_wheel.set(-RobotMap.littleAdjust);
								back_left_wheel.set(-RobotMap.littleAdjust);
								if (RobotMap.accTotal == 0) {
									front_right_wheel.set(0);
									back_right_wheel.set(0);
									front_left_wheel.set(0);
									back_left_wheel.set(0);
								}
								if (Math.abs(RobotMap.accTotal) > Math.abs(accStart)) {
									front_right_wheel.set(-RobotMap.littleAdjust);
									back_right_wheel.set(-RobotMap.littleAdjust);
									front_left_wheel.set(RobotMap.littleAdjust);
									back_left_wheel.set(RobotMap.littleAdjust);
									if (RobotMap.accTotal == 0) {
										front_right_wheel.set(0);
										back_right_wheel.set(0);
										front_left_wheel.set(0);
										back_left_wheel.set(0);
									}
									if (Math.abs(RobotMap.accTotal) > Math.abs(accStart)) {
										front_right_wheel.set(-RobotMap.littleAdjust);
										back_right_wheel.set(-RobotMap.littleAdjust);
										front_left_wheel.set(RobotMap.littleAdjust);
										back_left_wheel.set(RobotMap.littleAdjust);
										if (RobotMap.accTotal == 0) {
											front_right_wheel.set(0);
											back_right_wheel.set(0);
											front_left_wheel.set(0);
											back_left_wheel.set(0);
										}
										if (Math.abs(RobotMap.accTotal) > Math.abs(accStart)) {
											front_right_wheel.set(0);
											back_right_wheel.set(0);
											front_left_wheel.set(0);
											back_left_wheel.set(0);
										}
									}
								}
							}

							// Firing mechanism.
							if (leftStick.getRawButton(3)) {
								RobotMap.testShooterSpeed = RobotMap.testShooterSpeed - .05;
							}
							if (leftStick.getRawButton(4)) {
								RobotMap.testShooterSpeed = RobotMap.testShooterSpeed + .05;
							}
							if (leftStick.getTrigger()) {
								shooter.fire(RobotMap.testShooterSpeed);
							}
							// if(rightStick.getRawButton(2)) {
							// shooter.fire();
							// }

							/*
							if (leftStick.getRawButton(11)) {
								gear_tilt.set(.5);
							}
							if (leftStick.getRawButton(10)) {
								gear_deliver.set(-1);
							} else if (leftStick.getRawButton(9)) {
								gear_deliver.set(1);
							} else {
								gear_deliver.set(0);
							}
							*/

							/**
							 * Used for testing speed on the wheels.
							 */

							System.out.println("Speed of front right motor: " + rightFront.getRate());
							System.out.println("Speed of front left motor: " + leftFront.getRate());
							System.out.println("Speed of back right motor: " + rightBack.getRate());
							System.out.println("Speed of back left motor: " + leftBack.getRate());
							System.out.println("Speed of the shooting motor: " + shootWheel.getRate());

							/* Less voltage to motors */
							// myRobot.setMaxOutput(0.75);
							// Climber motor activated by button 2 on joystick
							/*
							 * if (rightStick.getRawButton(2)) { climber.set(1);
							 * climber2.set(1); } else { climber.set(0);
							 * climber2.set(0); }
							 */

							Scheduler.getInstance().run();
						}
					}
				}
			}
		} catch (Exception e) {
			System.err.println("Got exception:" + e.getMessage());
			e.printStackTrace();
		}
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
