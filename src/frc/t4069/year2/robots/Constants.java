package frc.t4069.year2.robots;

/**
 * Ports for devices.
 * 
 * @author Edmund
 */
public class Constants {

	public static final double DIST_BETWEEN_WHEELS = 43.18;

	/*
	 * PHYSICAL SECTION
	 */

	public static final double ZERO_INCLINE = 2.5;
	public static final double ZERO_SHOOTER_TURN = 6;

	/*
	 * ELECTRICAL SECTION
	 */

	// PWM ports
	public static final int RIGHT_MOTOR = 1;
	public static final int LEFT_MOTOR = 2;
	public static final int FRONT_SHOOTER = 7;
	public static final int BACK_SHOOTER = 9;
	public static final int INCLINER_VFD = 4;
	public static final int TURN_VFD = 6;

	// Analog

	public static final int INCLINER_POT = 2;
	public static final int TURN_POT = 1;

	// Joystick Ports
	public static final int GCPORT = 2;

	public static final int GCPORT2 = 1;

	// Digital
	// Solenoid
	public static final int RELOADER_SOLENOID_FORWARD = 1;
	public static final int RELOADER_SOLENOID_BACKWARD = 2;
	// Digital IO
	public static final int PRESSURE_SWITCH = 1;
	// Relay
	public static final int COMPRESSOR = 1;

	public static final int[] BAD_PORTS = {};

	private Constants() {
	}

}
