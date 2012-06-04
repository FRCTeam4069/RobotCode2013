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
	public static DriveTrain m_drivetrain;
	public static GameController m_gc;
	public static Joystick m_joystick;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		m_drivetrain = DriveTrain.getDriveTrain();
		m_gc = new GameController(new Joystick(RobotPorts.GCPORT));
		m_joystick = new Joystick(RobotPorts.JOYSTICKPORT);
		// For knowing whose code is on the robot.
		SmartDashboard.putString("Author", Version.author);
		SmartDashboard.putString("Version", Version.version);
		Logger.i("Robot initialized.");
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

	}

}
