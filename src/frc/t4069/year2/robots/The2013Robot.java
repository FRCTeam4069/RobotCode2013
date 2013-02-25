/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.t4069.year2.robots;

import java.util.Date;

import com.sun.squawk.debugger.Log;

import edu.wpi.first.wpilibj.AnalogModule;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.t4069.year2.robots.subsystems.DriveTrain;
import frc.t4069.year2.robots.subsystems.Shooter;
import frc.t4069.year2.utils.GameController;
import frc.t4069.year2.utils.Logger;

public class The2013Robot extends IterativeRobot {

	public static DriveTrain drivetrain;
	public static GameController drivegc;
	public static Shooter shooter;
	public static Joystick joystick;
	private static boolean ARCADE;
	private static double m_speedlimit = 1;
	private static double m_turnlimit = 0.7;

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {

	}

	boolean lastSlowButtonStatus = false;

	private void processDriveTrain(final boolean arcade) {
		if (arcade) {
			drivetrain.arcadeDrive(m_speedlimit * -drivegc.getTrigger(),
					drivegc.getLeftStick().x * m_turnlimit * m_speedlimit);
		} else {
			drivetrain.tankDrive(m_speedlimit * drivegc.getLeftStick().y,
					m_speedlimit * drivegc.getRightStick().y);
		}
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

	public void robotInit() {
		drivetrain = new DriveTrain();
		drivegc = new GameController(new Joystick(Constants.GCPORT));
		joystick = new Joystick(Constants.GCPORT2);
		shooter = new Shooter();
		// For knowing whose code is on the robot.
		SmartDashboard.putString("Author", Version.author);
		SmartDashboard.putString("Version", Version.version);
		Logger.i("Robot initialized.");
	}

	public void teleopInit() {
		ARCADE = m_ds.getDigitalIn(1);
		SmartDashboard.init();

		shooter.compress();

	}

	/**
	 * 
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		processShooter();
		processDriveTrain(ARCADE);
	}

	boolean shooting = false;
	Date shooterFinish = new Date();
	Date shooterStart = new Date();

	private void processShooter() {
		if (!joystick.getRawButton(1))
			shooter.load();
		else
			shooter.reload();
		if (joystick.getRawButton(3)) {
			shooter.shoot();
		} else
			shooter.shoot(0);
		 shooter.incline(joystick.getRawAxis(2));
		 shooter.turn
				( joystick.getRawButton(9) && !joystick.getRawButton(8)) ? 1 : joystick.getRawButton(8) ? -1 : 0);
	}
}