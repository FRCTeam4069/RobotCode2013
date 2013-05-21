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
        lastShot = null;
        frisbeesShot = 0;
    }
    int frisbeesShot = 0;

    public void autonomousPeriodic() {
        SmartDashboard.putNumber("Frisbees shot", frisbeesShot);
        if (lastShot != null) {
            SmartDashboard.putNumber("Last shot diff", -lastShot.getTime() + new Date().getTime());
        }
        // climber.gotoDriveAngle();
        if (frisbeesShot >= 3 && new Date().getTime() - lastShot.getTime() > 2500) {
            // shooter.inclinePID(8);
            drivetrain.tankDrive(-0.1, -0.1);
            return;
        }
        if (new Date().getTime() - autonomousStart.getTime() <= 5500) {
            shooter.load();
            shooter.inclinePID(m_ds.getDigitalIn(2) ? 27.4 : 27.2);
        } else {
            shooter.incline(0);
        }
        shooter.shoot();
        if (new Date().getTime() - autonomousStart.getTime() > 6000) {
            if (lastShot == null
                    || new Date().getTime() - lastShot.getTime() > 3000) {
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
            SmartDashboard.putNumber("Rotate value", drivegc.getLeftStick().x);
            SmartDashboard.putNumber("Speed value", drivegc.getTrigger());
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
        SmartDashboard.putString("Author", Version.author);
        SmartDashboard.putString("Version", Version.version);
        Log.log("Robot initialized.");
    }

    //public void disabledPeriodic() {
    //  climber.gotoAngle(90);
    //}
    public void teleopInit() {
        shooterStart = new Date();
        ARCADE = m_ds.getDigitalIn(1);
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
        processClimber();/*
         if (drivegc.getButton(GameController.BTN_B)) {
         shooter.inclinePID(13);
         shooter.turnPID(0);
         }*/
        // drivetrain.tankDrive(1, 1);
        // Log.log("Encoder: " + enc.get());
        SmartDashboard.putNumber("Tilter angle", shooter.getIncline());
        SmartDashboard.putNumber("Turret voltage", shooter.getTurnVoltage());
        SmartDashboard.putNumber("Turret angle", shooter.getTurn());
        SmartDashboard.putNumber("Climber voltage", climber.getAngle(true));

    }
    Date solenoidStart;
    boolean lastSolenoidButton = false;
    Date shooterStart = new Date();
    boolean lastIncreaseButton, lastDecreaseButton;
    private static final double FULL_COURT = 18.55;//17.72;//17.42;

    private void processShooter() {
        if (joystick.getRawButton(1) && !lastSolenoidButton) {
            solenoidStart = new Date();
        }
        if (solenoidStart != null
                && new Date().getTime() - solenoidStart.getTime() < 80) {
            shooter.reload();
            if (shooterStart != null) {
                shooterStart = new Date(shooterStart.getTime() + 2000);
            }
        } else {
            shooter.load();
        }
        lastSolenoidButton = joystick.getRawButton(1);
        SmartDashboard.putNumber("Time until up to speed", shooterStart == null ? 4 : Math.max(0, 4000 - (new Date().getTime() - shooterStart.getTime())) / 10);
        if (joystick.getRawButton(11)) {
            shooter.shoot(0.95, 1);
        } else if (joystick.getRawButton(2)) {
            if (shooterStart == null) {
                shooterStart = new Date();
            }
            if (new Date().getTime() - shooterStart.getTime() > 4000) {
                SmartDashboard.putString("At Speed", "?");
                SmartDashboard.putString("Spinning Up", "");
            } else {
                SmartDashboard.putString("At Speed", "");
                SmartDashboard.putString("Spinning Up", "?");
            }
            shooter.shoot();
        } else {
            SmartDashboard.putString("At Speed", "");
            SmartDashboard.putString("Spinning Up", "?");
            shooterStart = null;
            shooter.stop();
        }
        if (Math.abs(joystick.getRawAxis(2)) > 0.1) {
            shooter.unsetAngle();
            shooter.incline(joystick.getRawAxis(2));
            shooter.disableInclinePID();
            /* } else if (joystick.getRawButton(6) && !lastIncreaseButton) {
             Log.log("Button 6 pressed!");
             shooter.increaseAngle(1);
             } else if (joystick.getRawButton(5) && !lastDecreaseButton) {
             Log.log("Button 5 pressed!");
             shooter.decreaseAngle(1);
             */
        } else if (joystick.getRawButton(7)) {
            shooter.inclinePID(24);
        } else if (joystick.getRawButton(9)) {
            shooter.inclinePID(8);
        } else if (joystick.getRawButton(8)) {
            shooter.inclinePID(27.4);
        } else if (joystick.getRawButton(10)) {
            shooter.inclinePID(FULL_COURT);
            //     } else if (joystick.getRawButton(12)) {
            //       shooter.inclinePID(18);
        } else {
            //     if (!shooter.stabilize()) {
            shooter.incline(0);
            //}
        }
        if (Math.abs(joystick.getRawAxis(1)) > 0.1) {

            shooter.turn(joystick.getRawAxis(1));
        } else if (joystick.getRawButton(12)) {
            shooter.turnCentre();
        } else {
            shooter.turn(0);
        }
        lastIncreaseButton = joystick.getRawButton(6);
        lastDecreaseButton = joystick.getRawButton(5);


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

    public void disabledPeriodic() {
        SmartDashboard.putNumber("Tilter angle", shooter.getIncline());
        SmartDashboard.putNumber("Turret voltage", shooter.getTurnVoltage());
        SmartDashboard.putNumber("Turret angle", shooter.getTurn());
        SmartDashboard.putNumber("Climber voltage", climber.getAngle(true));
    }
}
