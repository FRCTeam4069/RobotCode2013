package frc.t4069.year2.robots.subsystems;

import java.util.Date;

import com.sun.squawk.util.MathUtils;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import frc.t4069.year2.robots.Constants;
import frc.t4069.year2.utils.math.LowPassFilter;

/**
 * RobotDrive is obviously too complicated, amirite? This class includes a low
 * pass filter.
 * 
 * Recommended setup is tank drive based robot, with 2 or 4 Jaguars (or Talons,
 * not victors). If 4 controllers are used, use a PWM splitter to split the PWM
 * signal. Controllers should be on brake mode.
 * 
 * Game controller (Logitech) should be mapped to trigger to be forward/reverse,
 * and left joystick's x to be left and right. These values are passed to
 * arcadeDrive as the move and turn value respectively.
 * 
 * @author Mostly Shuhao
 */
public class DriveTrain {

	private SpeedController m_leftESC;
	private SpeedController m_rightESC;
	private LowPassFilter m_leftLP;
	private LowPassFilter m_rightLP;
	private Encoder m_leftEnc;
	private Encoder m_rightEnc;
	private static final double MAX_SPEED = 657.14285714285714285714285714286;
	private double m_limit = 1.0;
	private double m_leftLimit = 0.95;
	private double m_rightLimit = 1.0;

	/**
	 * Initializes a new drive object with the RC value of 250.
	 */
	public DriveTrain() {
		this(100); // Good constant for this drive train
	}

	/**
	 * Initializes a new drive object with a custom RC value, using Jaguars.
	 * 
	 * @param RC
	 *            The RC value used for the drive train.
	 */

	public DriveTrain(double RC) {
		this(new Jaguar(Constants.LEFT_MOTOR),
				new Jaguar(Constants.RIGHT_MOTOR), RC);
	}

	/**
	 * Initializes a new drive train with all custom stuff. Talons are
	 * recommended above Jaguars, and Victors cannot be used as the low-pass
	 * filter doesn't work with their low response time and non-linearity. If
	 * there's 4 motors, use a PWM splitter. If you need to control all 4
	 * separately, this class is not for you.
	 * 
	 * @param leftESC
	 *            Left SpeedController object
	 * @param rightESC
	 *            Right SpeedController object
	 * @param RC
	 *            RC Constant for Low Pass Filter
	 */

	public DriveTrain(SpeedController leftESC, SpeedController rightESC,
			double RC) {
		this(leftESC, rightESC, null, null,// new Encoder(Constants.LEFT_ENC_1,
											// Constants.LEFT_ENC_2), new
											// Encoder(Constants.RIGHT_ENC_1,
											// Constants.RIGHT_ENC_2),
				RC);
	}

	class EncoderOutput implements PIDOutput {
		public double value = 0;

		public void pidWrite(double output) {
			value = output;

		}
	}

	public DriveTrain(SpeedController leftESC, SpeedController rightESC,
			Encoder leftEncoder, Encoder rightEncoder, double RC) {
		m_leftESC = leftESC;
		m_rightESC = rightESC;
		m_leftEnc = leftEncoder;
		m_rightEnc = rightEncoder;
		m_leftLP = new LowPassFilter(RC);
		m_rightLP = new LowPassFilter(RC);
	}

	/**
	 * Mathematical arcade drive. It calculates the left and right speed.
	 * 
	 * @param moveValue
	 *            Value between -1 - 1
	 * @param rotateValue
	 *            Value between -1 - 1
	 */
	public void arcadeDrive(double moveValue, double rotateValue) {
		double leftMotorSpeed;
		double rightMotorSpeed;
		double theta = MathUtils.atan2(moveValue, rotateValue);
		double r = Math.sqrt((moveValue * moveValue) + (rotateValue * rotateValue));
		leftMotorSpeed = (Math.sin(theta) + Math.cos(theta)) * r;
		rightMotorSpeed = (Math.sin(theta) - Math.cos(theta)) * r;
		tankDrive(leftMotorSpeed, rightMotorSpeed);
	}

	/**
	 * Gets distance traveled since last reset. Will return an incorrect value
	 * if any turning has taken place s'ince the last reset.
	 */
	public double getDistance() {
		double value = (m_leftEnc.getDistance() + m_rightEnc.getDistance()) / 2;
		resetEncoders();
		return value;
	}

	/**
	 * Gets the current RC value;
	 * 
	 * @return The current RC value set.
	 */
	public double getRC() {
		return m_leftLP.getRC();
	}

	/**
	 * Gets angle turned.
	 * 
	 * @param degrees
	 *            If true, returns angle in degrees. Otherwise, returns radians.
	 */

	public double getTurnedAngle(boolean degrees) {
		double circumference = 2 * Constants.DIST_BETWEEN_WHEELS * Math.PI;
		double leftDist = m_leftEnc.getDistance();
		double rightDist = m_rightEnc.getDistance();
		double arc = rightDist - leftDist;
		if (degrees) {
			return (arc / circumference) * 360;
		}
		return arc / Constants.DIST_BETWEEN_WHEELS;
	}

	/**
	 * Stops robot by setting the speed of the controller to 0 (remember that
	 * the ESC should be on brake mode)
	 */
	public void hardBreak() {
		m_leftESC.set(0);
		m_rightESC.set(0);
	}

	/**
	 * Limit the left side, used for if one side is more powerful. You can use
	 * this to reverse directions (negatives)
	 * 
	 * @param limit
	 *            A double between -1 - 1, as a percentage
	 */
	public void limitLeft(double limit) {
		m_leftLimit = limit;
	}

	/**
	 * Limit the left side, used for if one side is more powerful. You can use
	 * this to reverse directions (negatives)
	 * 
	 * @param limit
	 *            A double between -1 - 1, as a percentage
	 */
	public void limitRight(double limit) {
		m_rightLimit = limit;
	}

	/**
	 * Limit the max speed. Used for precision mode. You can also use this to
	 * reverse directions (negatives)
	 * 
	 * @param limit
	 *            A double between -1 - 1, as a percentage
	 */
	public void limitSpeed(double limit) {
		m_limit = limit;
	}

	private void resetEncoders() {
		m_rightEnc.reset();
		m_leftEnc.reset();
	}

	/**
	 * Sets a RC value. Used when tuning with the analog controls.
	 * 
	 * @param RC
	 *            the RC value
	 */
	public void setRC(double RC) {
		m_leftLP.setRC(RC);
		m_rightLP.setRC(RC);
	}

	/**
	 * Tank drive. Controls the left and right speed. Not used usually.
	 * 
	 * @param leftSpeed
	 *            Left speed between -1 - 1
	 * @param rightSpeed
	 *            Right speed between -1 - 1
	 */

	public void tankDrive(double leftSpeed, double rightSpeed) {
		leftSpeed *= m_leftLimit * -m_limit;
		rightSpeed *= m_rightLimit * m_limit;
		leftSpeed = m_leftLP.calculate(leftSpeed);
		rightSpeed = m_rightLP.calculate(rightSpeed);
		m_leftESC.set(leftSpeed);
		m_rightESC.set(rightSpeed);
	}

}
