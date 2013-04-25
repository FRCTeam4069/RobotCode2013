package frc.t4069.year2.utils;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.AnalogModule;
import frc.t4069.year2.robots.Constants;

public class Potentiometer extends Object {

    private int m_potChannel;
    private double zeroVoltage, slope;

    public Potentiometer(int channel, double degreeIntercept, double slope) {
        m_potChannel = channel;
        this.zeroVoltage = degreeIntercept;
        this.slope = slope;
        AnalogModule.getInstance(AnalogModule.getDefaultAnalogModule()).setOversampleBits(channel, 6);
        AnalogModule.getInstance(AnalogModule.getDefaultAnalogModule()).setSampleRate(100000);
    }

    public double getAngle(boolean degrees) {
        double degs = (getVoltage() * slope) + zeroVoltage;
        return -degs;
    }

    public double getVoltage() {
        return AnalogModule.getInstance(AnalogModule.getDefaultAnalogModule())
                .getAverageVoltage(m_potChannel);
    }
}
