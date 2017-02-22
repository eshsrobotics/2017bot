package org.usfirst.frc.team1759.robot.commands;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.List;

import com.ctre.CANTalon;

public class CreateGenerateControllersCommand {

	public List<CANTalon> createTalons(int talonSizeBegin, int talonSizeEnd) {
		List<CANTalon> talons = new ArrayList<CANTalon>(talonSizeEnd);
		for (int i = talonSizeBegin; i < talonSizeEnd; ++i) {
			talons.set(i, new CANTalon(i));
		}

		return talons;
	}

	public Dictionary<String, CANTalon> generateTalons() {
		// Loop through list of talons, creating an entry for each talon
	}

}
