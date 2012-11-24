
package frc.t4069.year2.robots;

/**
 * Ports for devices.
 * 
 * @author Edmund
 */
public class Constants {

	public static final double DIST_BETWEEN_WHEELS = 52.705;

	/*
	 * PHYSICAL SECTION
	 */

	public static final double WHEEL_CIRCUMFERENCE = Math.PI * 17.78;

	// PWM ports
	public static final int RIGHT_MOTOR = 1;
	public static final int LEFT_MOTOR = 2;

	/*
	 * ELECTRICAL SECTION
	 */

	// Analog

	// Joystick Ports
	public static final int GCPORT = 1;

	public static final int JOYSTICKPORT = 2;
	// Encoders
	public static final int LEFT_ENC_1 = 1;

	// Digital

	public static final int LEFT_ENC_2 = 2;
	public static final int RIGHT_ENC_1 = 3;
	public static final int RIGHT_ENC_2 = 4;
	public static final double PULSES_PER_REVOLUTION = 1000;

	private Constants() {
	}

}
