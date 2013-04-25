/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.t4069.year2.robots.subsystems;

import com.sun.squawk.debugger.Log;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.AnalogModule;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import frc.t4069.year2.robots.Constants;
import frc.t4069.year2.utils.Potentiometer;

/**
 *
 * @author Developer
 */
public class Winch {

    protected Potentiometer m_winchPot;
    protected Relay m_winchRelay;

    public Winch() {
        this(new Potentiometer(Constants.WINCH_POT, 1, 1),
                new Relay(Constants.WINCH_CONTROLLER));
    }

    public double getAngle(boolean degrees) {
        return m_winchPot.getAngle(degrees);
    }

    public Winch(Potentiometer winchPot, Relay winchRelay) {
        m_winchPot = winchPot;
        m_winchRelay = winchRelay;
//        m_winchpidsource = new Shooter.AnglePotentiometer(winchPot);
//        m_winchpidoutput = new Shooter.Output();
//        m_winchpc = new PIDController(winchp, winchi, winchd, m_winchpidsource, m_winchpidoutput);
//        m_winchpc.setOutputRange(-1, 1);
    }
// Not necessary, apparently. Keeping it for the future. DO NOT USE until 
// the pot is calibrated.

    public void gotoAngle(double angle) {
        if (Math.abs(m_winchPot.getAngle(true) - angle) < 0.5) {
            return;
        }
        if (m_winchPot.getAngle(true) > angle) {
            lower();
        } else {
            raise();
        }
    }
    // Goes to the angle for driving.

    public void gotoDriveAngle() {
        gotoAngle(120);
    }

    public void raise() {
        //     if (m_winchPot.getAverageVoltage() >= 0.42) {
        if (m_winchRelay.get() != Relay.Value.kReverse)
        m_winchRelay.set(Relay.Value.kReverse);
        //   } else {
        //      m_winchVFD.set(Relay.Value.kOff);
        // }
    }
    //  int ARMS_BACK = -1, ARMS_MIDDLE = 0, ARMS_MIDDLE = 1;
//public WinchStatus getStatus() {if
//(    m_winchPot.getAverageVoltage() > 2.2) {}
//}

    public void lower() {
        // if (m_winchPot.getAverageVoltage() <= 2.2) {
        if (m_winchRelay.get() != Relay.Value.kForward)
        m_winchRelay.set(Relay.Value.kForward);
        // } else {
        //    m_winchVFD.set(Relay.Value.kOff);
    }

    //  0.38 V
//2.3561 V
    public void stop() {
        //      Log.log("" + m_winchPot.getAverageVoltage());
        if (m_winchRelay.get() != Relay.Value.kOff)
        m_winchRelay.set(Relay.Value.kOff);
    }
}
