/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.t4069.year2.robots.subsystems;

import com.sun.squawk.debugger.Log;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.AnalogModule;
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

    protected AnalogChannel m_winchPot;
    protected Relay m_winchVFD;

    public Winch() {
        this(new AnalogChannel(Constants.WINCH_POT),//new Potentiometer(Constants.WINCH_POT,1,1), 
                new Relay(Constants.WINCH_CONTROLLER));

    }

    public double getAngle(boolean degrees) {
        return m_winchPot.getAverageVoltage();
    }

    public Winch(AnalogChannel winchPot, Relay winchVFD) {
        m_winchPot = winchPot;
        m_winchPot.setOversampleBits(2);
        m_winchVFD = winchVFD;
    }

    public void raise() {
        //     if (m_winchPot.getAverageVoltage() >= 0.42) {
        m_winchVFD.set(Relay.Value.kReverse);
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
        m_winchVFD.set(Relay.Value.kForward);
        // } else {
        //    m_winchVFD.set(Relay.Value.kOff);
    }

    //  0.38 V
//2.3561 V
    public void stop() {
        Log.log("" + m_winchPot.getAverageVoltage());
        m_winchVFD.set(Relay.Value.kOff);
    }
}
