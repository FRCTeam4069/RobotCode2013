/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.t4069.year2.robots;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.t4069.year2.robots.subsystems.DriveTrain;
import frc.t4069.year2.utils.GameController;
import frc.t4069.year2.utils.Logger;

public class The2013Robot extends IterativeRobot {

	public static DriveTrain drivetrain;
	public static GameController gc;
	public static Joystick joystick;
	private static boolean ARCADE;
	private static double m_speedlimit = 1;

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {

	}

	private void processDriveTrain(final boolean arcade) {
		if (gc.getButton(GameController.BTN_RB)
				|| gc.getButton(GameController.BTN_LB)) {
			drivetrain.hardBreak();
		}
		else if (arcade) {
			drivetrain.arcadeDrive(m_speedlimit * gc.getTrigger(),
					m_speedlimit * gc.getLeftStick().x);
		}
		else {
			drivetrain.tankDrive(m_speedlimit * gc.getLeftStick().y,
					m_speedlimit * gc.getRightStick().y);
		}
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		drivetrain = new DriveTrain();
		gc = new GameController(new Joystick(Constants.GCPORT));
		joystick = new Joystick(Constants.JOYSTICKPORT);
		// For knowing whose code is on the robot.
		SmartDashboard.putString("Author", Version.author);
		SmartDashboard.putString("Version", Version.version);
		Logger.i("Robot initialized.");
	}

	public void teleopInit() {
		ARCADE = m_ds.getDigitalIn(1);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		if (gc.getButton(GameController.BTN_BACK)) {
			m_speedlimit = 0.6;
		}
		else {
			m_speedlimit = 1;
		}
		processDriveTrain(ARCADE);
	}

}
