/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.t4069.year2.robots;

import java.util.Date;

import com.sun.squawk.debugger.Log;
import edu.wpi.first.wpilibj.AnalogChannel;

import edu.wpi.first.wpilibj.AnalogModule;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalModule;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.t4069.year2.robots.subsystems.DriveTrain;
import frc.t4069.year2.robots.subsystems.Shooter;
import frc.t4069.year2.robots.subsystems.Winch;
import frc.t4069.year2.utils.GameController;

public class The2013Robot extends IterativeRobot {

    public static DriveTrain drivetrain;
    public static GameController drivegc;
    public static Shooter shooter;
    public static Winch climber;
    public static Joystick joystick;
    private static boolean ARCADE;
    private static double m_speedlimit = 1;
    private static double m_turnlimit = 0.7;
    /**
     * This function is called periodically during autonomous
     */
    private Date autonomousStart;
    private Date lastShot;

    public void autonomousInit() {
        autonomousStart = new Date();
    }
    int frisbeesShot = 0;

    public void autonomousPeriodic() {
//SmartDashboard.putNumber("Frisbees shot", frisbeesShot);
//SmartDashboard.putString("Last shot", lastShot.toString());
        if (frisbeesShot >= 3 && new Date().getTime() - lastShot.getTime() > 2500) {
           // shooter.inclinePID(8);
            return;
        }
        shooter.inclinePID(26.8);
        shooter.shoot();
        if (new Date().getTime() - autonomousStart.getTime() > 4000) {
            if (lastShot == null
                    || new Date().getTime() - lastShot.getTime() > 1900) {
                frisbeesShot++;
                shooter.reload();
                lastShot = new Date();
            } else if (lastShot != null
                    && new Date().getTime() - lastShot.getTime() > 250) {
                shooter.load();
            }
        }
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
        drivegc = new GameController(new Joystick(Constants.GCPORT2));
        joystick = new Joystick(Constants.GCPORT);
        shooter = new Shooter();
        climber = new Winch();
        // For knowing whose code is on the robot.
        //SmartDashboard.putString("Author", Version.author);
        //SmartDashboard.putString("Version", Version.version);
        Log.log("Robot initialized.");
        // backChannel1 = new DigitalInput(5);
        // backChannel2 = new DigitalInput(6);
        // enc = new Encoder(5,6);
    }

    public void teleopInit() {
        ARCADE = m_ds.getDigitalIn(1);
        // shooter.compress();

    }
    Thread debugThread = new Thread(new Runnable() {
        public void run() {
        }
    });

    /**
     *
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        // Log.log("Front wheel: " + frontEncoder.getRaw());
        // Log.log("Back wheel: " + backEncoder.getRaw());

        // Log.log("Channel A: " + backChannel1.get());
        // Log.log("Channel B: " + backChannel2.get());
        // shooterFront.set(-0.5);
        // Log.log("PID OUT:  " + shooter.getPIDFront());
        processShooter();
        // Log.log("" + shooter.getIncline());
        // processShooter();
        processDriveTrain(ARCADE);
        processClimber();
        // drivetrain.tankDrive(1, 1);
        // Log.log("Encoder: " + enc.get());
        SmartDashboard.putNumber("Tilter angle", shooter.getIncline());
        SmartDashboard.putNumber("Turret voltage", shooter.getTurnVoltage());
        SmartDashboard.putNumber("Turret angle", shooter.getTurn());
        SmartDashboard.putNumber("Climber voltage", climber.getAngle(true));
    }
    Date solenoidStart;
    int count = 0;
    boolean lastSolenoidButton = false;
    boolean lastStabilizeButton = false;

    private void processShooter() {
        if (joystick.getRawButton(1) && !lastSolenoidButton) {
            solenoidStart = new Date();
        }
        if (solenoidStart != null
                && new Date().getTime() - solenoidStart.getTime() < 80) {
            shooter.reload();
        } else {
            shooter.load();
        }
        lastSolenoidButton = joystick.getRawButton(1);
        if (joystick.getRawButton(6)) {
            if (!lastStabilizeButton) {
                shooter.setAngle();
            }
        }

        lastStabilizeButton = joystick.getRawButton(6);
        if (joystick.getRawButton(11)) {
            shooter.shoot(0.95, 1);
        } else if (joystick.getRawButton(2)) {
            shooter.shoot();
        } else {
            shooter.stop();
        }
        if (Math.abs(joystick.getRawAxis(2)) > 0.01 || shooter.isAngleSet()) {
            shooter.unsetAngle();
            shooter.incline(joystick.getRawAxis(2));
            shooter.disableInclinePID();
        } else if (joystick.getRawButton(7)) {
            shooter.inclinePID(24);
        } else if (joystick.getRawButton(9)) {
            shooter.inclinePID(8);
        } else if (joystick.getRawButton(8)) {
            shooter.inclinePID(26.8);
        } else if (joystick.getRawButton(10)) {
            shooter.inclinePID(17);
        } else if (joystick.getRawButton(12)) {
            shooter.inclinePID(18);
        } else {
           if (! shooter.stabilize()) shooter.incline(0);
        }
        if (count % 5 == 0) {
       //     Log.log("Inclination: " + shooter.getIncline() + "Yaw: "
         //           + shooter.getTurn());
        }

        count += (count > 300000 ? -count : 1);
        shooter.turn(joystick.getRawAxis(1));
        // shooter.turn
        // ( joystick.getRawButton(9) && !joystick.getRawButton(8)) ? 1 :
        // joystick.getRawButton(8) ? -1 : 0);
    }

    private void processClimber() {
        if (joystick.getRawButton(3)) {
            climber.lower();
        } else if (joystick.getRawButton(4)) {
            climber.raise();
        } else {
            climber.stop();
        }
    }
}
