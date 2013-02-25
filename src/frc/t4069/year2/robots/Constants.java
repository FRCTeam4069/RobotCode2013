package frc.t4069.year2.robots;

import edu.wpi.first.wpilibj.DigitalSource;

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

	public static final double WHEEL_CIRCUMFERENCE = Math.PI * 15.24;

	// PWM ports
	public static final int RIGHT_MOTOR = 1;
	public static final int LEFT_MOTOR = 2;
	public static final int FRONT_SHOOTER = 3;
	public static final int BACK_SHOOTER = 4;
	/*
	 * ELECTRICAL SECTION
	 */

	// Analog

	// Joystick Ports
	public static final int GCPORT = 1;

	public static final int GCPORT2 = 2;

	// Digital

	// Encoders
	public static final int LEFT_ENC_1 = 1;
	public static final int LEFT_ENC_2 = 2;
	public static final int RIGHT_ENC_1 = 3;
	public static final int RIGHT_ENC_2 = 4;
	public static final double PULSES_PER_DT_REVOLUTION = 1000; // pulses per
																// revolution
																// (DriveTrain)
	public static final int SHOOTER_ENCODER = 5;
	public static final double PULSES_PER_ST_REVOLUTION = 1000; // pulses per
																// revolution
																// (Shooter)
	public static final double ZERO_INCLINE = 2.5;
	public static final double ZERO_SHOOTER_TURN = 6;
	public static final int INCLINER_VFD = 5;
	public static final int INCLINER_POT = 2;
	public static final int TURN_POT = 1;

	public static final int RELOADER_SOLENOID_FORWARD = 1;
	public static final int RELOADER_SOLENOID_BACKWARD = 2;

	public static final int SHOOTER_ENCODER_2 = 6;

	public static final int PRESSURE_SWITCH = 7;
	public static final int COMPRESSOR = 8;

	public static final int TURN_VFD = 6;

	private Constants() {
	}

}
