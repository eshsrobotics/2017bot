 
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
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

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
	public static final double thresholdX = .35;		//Added to make sure the drive isn't too sensitive
	public static final double thresholdY = .2;			//As Above
	public static final double thresholdTwist = .2;		//As Above
	public static final double low = .45;				//Added to lower speed for precision
	public static final double medium = .65;			//Added to lower speed for power saving
	public static final double high = .75;				//Added to limit speed slightly
	public static final int max = 1;					//Added because it made everything easier to do code wise.
	public static double testShooterSpeed = .5;			//Used to test shooter speed to determine best distance.
	public static double accX = 0;						//Accleration in the X-direction
	public static double accY = 0;						//Acceleration in the Y-direction
	public static double accZ = 0;						//Acceleration in the Z-direction
	public static double accTotal = 0;					//Total Acceleration, as read by the accelerometer
	public static final double littleAdjust = 0.1;		//For making little adjustments with the accelerometer code.
	//public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

	Command autonomousCommand;
	SendableChooser chooser;
	String autoSelected;

	RobotDrive myRobot;
	Joystick leftStick;
	Joystick rightStick;
	Joystick shootStick;
	CANTalon shoot;
	CANTalon shoot2;
	CANTalon back_right_wheel;
	CANTalon front_right_wheel;
	CANTalon back_left_wheel;
	CANTalon front_left_wheel;
	Encoder rightBack;
	Encoder rightFront;
	Encoder leftBack;
	Encoder leftFront;
	Encoder shooting;
	ADXRS450_Gyro gyro;
	BuiltInAccelerometer accel;
	XMLParser xmlParser;
	PapasData papasData;
	ServerRunnable serverRunnable;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		oi = new OI(); // TODO: OI.java see if neccessary.
		
		// Define Autonomous mode options and display on Driver station dash.
		chooser = new SendableChooser();
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		xmlParser = new XMLParser();
		papasData = new PapasData();
		serverRunnable = new ServerRunnable();

		// Initalize talons.
		CANTalon talons[] = new CANTalon[9];
		for (int i = 0; i < talons.length; ++i) {
			talons[i] = new CANTalon(i);
		}
		
		
		gyro = new ADXRS450_Gyro();
		gyro.reset();
		gyro.calibrate();

		/*
		 If you draw an imaginary "И" (Cyrillic ee) on the top of the robot
		 starting from the front left wheel, the "И" will end with the back
		 right wheel and will hit the talons in numerical order.
		*/
		front_left_wheel = talons[0];
		back_left_wheel = talons[1];
		front_right_wheel = talons[2];
		back_right_wheel = talons[3];
		shoot = talons[4];
		shoot2 = talons[5];
		

		// Inverting signal since they are wired in reverse polarity on the robot
		talons[0].setInverted(true);
		talons[1].setInverted(true);
		talons[2].setInverted(false);
		talons[3].setInverted(false);

		// front left, back left, front right, back right
		myRobot = new RobotDrive(front_left_wheel, back_left_wheel, front_right_wheel, back_right_wheel);

		/* 
		load talon port (cantalon), lower shoot talon port(cantalon), upper
		shoot talon port(cantalon)
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
			leftFront.setMaxPeriod(.1);
			leftFront.setMinRate(10);
			leftFront.setDistancePerPulse(5);
			leftFront.setReverseDirection(false);
			leftFront.setSamplesToAverage(7);
		Encoder shoot = new Encoder(8, 9, false, CounterBase.EncodingType.k2X);
			shoot.setMaxPeriod(.1);
			shoot.setMinRate(10);
			shoot.setDistancePerPulse(5);
			shoot.setReverseDirection(false);
			shoot.setSamplesToAverage(7);
		
		accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);
		accX = accel.getX();
		accY = accel.getY();
		accZ = accel.getZ();
		accTotal = Math.sqrt((accX*accX) + (accZ*accZ));
		
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
		 This makes sure that the autonomous stops running when
		 teleop starts running. If you want the autonomous to
		 continue until interrupted by another command, remove
		 this line or comment it out.
		 */
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

    // TODO: Enter gyro angle reading into last parameter.
		double Kp = 0.03;
		double angle = gyro.getAngle();
		double rightStickX = 0;
		double rightStickY = 0;
		double rightStickTwist = 0;
		double accStart = 0;
		myRobot.setMaxOutput(medium);
		
		if(xmlParser.parse(papasData.solutionFound).equals("true")){
			if(xmlParser.parse(papasData.solutionType).equals("boiler")){
				
			}
			if(xmlParser.parse(papasData.solutionType).equals("peg")){
				
			}
		}
			
		if( Math.abs(rightStick.getX()) > thresholdX){
			rightStickX = rightStick.getX();
		}
		if( Math.abs(rightStick.getY()) > thresholdY){
			rightStickY = rightStick.getY();
		}
		if( Math.abs(rightStick.getTwist()) > thresholdTwist){
			rightStickTwist = rightStick.getTwist();
		}
		if(rightStick.getRawButton(5)){
			front_right_wheel.set(max);
			front_left_wheel.set(max);
			back_right_wheel.set(max);
			back_left_wheel.set(max);
		}
		if(rightStick.getRawButton(3)){
			front_right_wheel.set(high);
			front_left_wheel.set(high);
			back_right_wheel.set(high);
			back_left_wheel.set(high);
		}
		if(rightStick.getRawButton(4)){
			front_right_wheel.set(low);
			front_left_wheel.set(low);
			back_right_wheel.set(low);
			back_left_wheel.set(low);
		}
		myRobot.mecanumDrive_Cartesian(rightStickY, -rightStickX, -rightStickTwist, angle*Kp);
		//myRobot.mecanumDrive_Cartesian(rightStick.getY(), rightStick.getX(), rightStick.getTwist(), 0);
		
		if(rightStickX == 0 && rightStickY == 0 && rightStickTwist == 0){
			if(accTotal != 0){
				front_right_wheel.set(littleAdjust);
				back_right_wheel.set(littleAdjust);
				front_left_wheel.set(-littleAdjust);
				back_left_wheel.set(-littleAdjust);
				if(accTotal == 0){
					front_right_wheel.set(0);
					back_right_wheel.set(0);
					front_left_wheel.set(0);
					back_left_wheel.set(0);
				}
				if(Math.abs(accTotal) > Math.abs(accStart)){
					front_right_wheel.set(-littleAdjust);
					back_right_wheel.set(-littleAdjust);
					front_left_wheel.set(littleAdjust);
					back_left_wheel.set(littleAdjust);
					if(accTotal == 0){
						front_right_wheel.set(0);
						back_right_wheel.set(0);
						front_left_wheel.set(0);
						back_left_wheel.set(0);
					}
					if(Math.abs(accTotal) > Math.abs(accStart)){
						front_right_wheel.set(0);
						back_right_wheel.set(0);
						front_left_wheel.set(0);
						back_left_wheel.set(0);
					}
				}
			}
		}
		
		while(rightStick.getRawButton(10) == true){
			if(rightStick.getRawButton(8)){
				testShooterSpeed = testShooterSpeed + 1;
				System.out.println(testShooterSpeed);
			}
			if(rightStick.getRawButton(7)){
				testShooterSpeed = testShooterSpeed - 1;
				System.out.println(testShooterSpeed);
			}
			if(rightStick.getRawButton(9));{
				shoot.set(testShooterSpeed);
				shoot2.set(testShooterSpeed);
			}
		}


		Scheduler.getInstance().run();
	}
 
	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}
}
