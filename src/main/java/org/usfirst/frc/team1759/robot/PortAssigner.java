package org.usfirst.frc.team1759.robot;

import java.util.Dictionary;
import java.util.Hashtable;

public class PortAssigner {

		public static String LEFT_FRONT_WHEEL = "front_left_wheel";
		public static String RIGHT_FRONT_WHEEL = "front_right_wheel";
		public static String RIGHT_BACK_WHEEL = "back_right_wheel";
		public static String LEFT_BACK_WHEEL = "back_left_wheel";
		public static String SHOOT_WHEEL = "shoot_wheel";
		public static String FEED_WHEEL = "feed_wheel";
	
	/**
	 * 
	 * @author Aidan Galbreath and Ari Berkowicz
	 *
	 **/

	private Dictionary<String, Integer> portTable = new Hashtable<String, Integer>();
	private static PortAssigner theInstance;

	private PortAssigner() {
		portTable.put(LEFT_FRONT_WHEEL, 0);
		portTable.put(RIGHT_FRONT_WHEEL, 2);
		portTable.put(LEFT_BACK_WHEEL, 1);
		portTable.put(RIGHT_BACK_WHEEL, 3);
		portTable.put(SHOOT_WHEEL, 4);
		portTable.put(FEED_WHEEL, 5);
	}

	public int getAssignedPort(String controllerName) {
		return portTable.get(controllerName);
	}

	public static PortAssigner getInstance() {
		return theInstance;
	}

}
