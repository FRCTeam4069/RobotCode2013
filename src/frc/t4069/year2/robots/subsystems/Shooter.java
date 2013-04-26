package frc.t4069.year2.robots.subsystems;

import java.util.Date;

import com.sun.squawk.debugger.Log;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.t4069.year2.robots.Constants;
import frc.t4069.year2.utils.Potentiometer;
import frc.t4069.year2.utils.math.LowPassFilter;

public class Shooter {

    public void setAngle() {
        setAngle(getIncline());
    }

    static class Output implements PIDOutput {

        public double value = 0;

        public void pidWrite(double output) {
            value = output;
        }
    }

    static class RPMEncoder implements PIDSource {

        private Encoder m_encoder;
        private long lastTime = -1337;
        private int lastValue = 0;
        private double currentValue = 0.0;
        private LowPassFilter m_lpf = new LowPassFilter(25);

        public RPMEncoder(Encoder encoder) {
            m_encoder = encoder;
        }

        public double pidGet() {
            if (lastTime == -1337) {
                lastTime = new Date().getTime();
                lastValue = 0;
                return 0;
            }
            long ct = new Date().getTime();
            double deltaTime = ct - lastTime;
            lastTime = ct;
            int thisValue = m_encoder.get();
            int deltaValue = thisValue - lastValue;
            lastValue = thisValue;
            if (lastValue > 2000000000) {
                m_encoder.reset();
                lastValue = 0;
            }
            double rev = deltaValue / 30.0;
            rev = rev / (deltaTime / 60000.0);// / MAX_SPEED;
            //Log.log("RPM: " + rev);
            currentValue = m_lpf.calculate(rev);
            return currentValue;
        }
    }
    protected SpeedController m_VFDBack;
    protected SpeedController m_VFDFront;
    protected LowPassFilter m_backLPF, m_frontLPF;
    public boolean fastMode = false;
    protected SpeedController m_Turret, m_Incliner;
    protected DoubleSolenoid m_Reloader;
    protected Compressor m_Compressor;
    protected Potentiometer m_TurningPot, m_RisingPot;
    protected Encoder m_BackEncoder, m_FrontEncoder;
    private PIDController m_turnPC;
    private PIDController m_inclinePC;
    private PIDController m_FrontShootPC;
    private PIDSource m_FrontShootPIDSource;
    private PIDOutput m_FrontShootPIDOutput;
    private PIDSource m_inclinepidsource, m_turnpidsource;
    private PIDOutput m_inclinepidoutput, m_turnpidoutput;
    private double inclined = 0,//0 initial constant
            inclinei = 0.0013//0.0012//0.0009//0.0009 //0.005 initial constant 
            , inclinep = 0.05; //0.15 initial constant
    private double turnd = 0, turni = 0.0001, turnp =0.3;
    private LowPassFilter m_InclineLPF = new LowPassFilter(50);
    private LowPassFilter m_TurnLPF = new LowPassFilter(10);

    public Shooter() {
        this(45);
    }

    public Shooter(double RC) {
        this(new Jaguar(Constants.FRONT_SHOOTER), new Jaguar(
                Constants.BACK_SHOOTER), new Jaguar(Constants.INCLINER_VFD),
                new Victor(Constants.TURN_VFD), new DoubleSolenoid(
                Constants.RELOADER_SOLENOID_FORWARD,
                Constants.RELOADER_SOLENOID_BACKWARD), new Compressor(
                Constants.PRESSURE_SWITCH, Constants.COMPRESSOR),
                Constants.TURN_POT, Constants.INCLINER_POT, RC);
    }

    /**
     * Creates a new Shooter object. Note: The use of Talons for shooter control
     * is recommended. Seriously, don`t use Victors. Ever.
     */
    public Shooter(SpeedController escFront, SpeedController escBack,
            SpeedController incliner, SpeedController turner,
            DoubleSolenoid reloader, Compressor compressor, int turnPot,
            int risePot, double RC) {
        m_VFDBack = escBack;
        m_VFDFront = escFront;
        m_Reloader = reloader;
        m_Compressor = compressor;
        m_Incliner = incliner;
        m_Turret = turner;
        m_TurningPot = new Potentiometer(turnPot, 0, -1); // not using an actual angle, 
        // because we're too lazy to calibrate it

        m_RisingPot = new Potentiometer(risePot, -263.150, 92.0444649);
        m_inclinepidsource = m_RisingPot;
        m_inclinepidoutput = new Output();
        m_turnpidsource = m_TurningPot;
        m_turnpidoutput = new Output();
        m_inclinePC = new PIDController(inclinep, inclinei, inclined,
                m_inclinepidsource, m_inclinepidoutput, 0.050); //(,,0.025)
        m_inclinePC.setOutputRange(-0.59, 0.59);
        m_inclinePC.setInputRange(0, 39);
        m_turnPC = new PIDController(turnp, turni, turnd, m_turnpidsource, m_turnpidoutput, 0.050);
        m_turnPC.setInputRange(0.7, 1.9);
        m_turnPC.setOutputRange(-1, 1);
        //  m_inclinePC.setTolerance(1.5);
       // m_turnPC.setAbsoluteTolerance(0.03);
        m_turnPC.setAbsoluteTolerance(0.03);
        m_inclinePC.setAbsoluteTolerance(0.37);
        m_Compressor.start();
    }

    public double getIncline() {
        return m_RisingPot.getAngle(true);
    }

    public double getIncline(boolean degrees) {
        return m_RisingPot.getAngle(degrees);
    }

    public void increaseAngle(double angle) {
        SmartDashboard.putNumber("Set angle", setAngle);
        if (setAngle < 0) {
            setAngle = getIncline() + angle;
        } else {
            setAngle += angle;
            m_inclinePC.reset();
            m_inclinePC.enable();
        }
    }

    public void decreaseAngle(double angle) {
        increaseAngle(-angle);
    }

    public boolean getPressureSwitch() {
        return m_Compressor.getPressureSwitchValue();
    }

    public void inclineChecked(double speed) {
        //Log.log("Speed: " + speed);
        if (m_RisingPot.getAngle(true) <= 11) {
            if (speed > 0) {
                m_Incliner.set(0);
                return;
            }
        }
        if (m_RisingPot.getAngle(true) >= 39) {
            if (speed < 0) {
                m_Incliner.set(0.1);
                return;
            }
        }
        m_Incliner.set(speed);
    }
    double setAngle = -1;

    public void setAngle(double angle) {
        setAngle = angle;
        SmartDashboard.putNumber("Set angle", angle);
    }

    public boolean stabilize() {
        SmartDashboard.putNumber("Set angle", setAngle);
        if (setAngle < 0) {
            return false;
        }
        inclinePID(setAngle);
        return true;
    }

    public void unsetAngle() {
        setAngle = -1;
    }

    public double getTurn() {
        return getTurn(true);// return m_TurningPot.getAngle(true);
    }

    public double getTurnVoltage() {
        return m_TurningPot.getVoltage();
    }

    public double getTurn(boolean degrees) {
        return m_TurningPot.getAngle(degrees);
    }

    public void incline(double speed) {
        inclineChecked(-m_InclineLPF.calculate(speed * 0.55));

    }

    public void incline(double angle, boolean degrees) {
        if (Math.abs(1 - angle / m_RisingPot.getAngle(degrees)) < 0.05) {
            m_inclinePC.disable();
            return;
        }
    }

    public boolean isAngleSet() {
        return setAngle > 0;
    }

    public void inclinePID(double target) {
        if (m_inclinePC.getSetpoint() != target) {
            m_inclinePC.reset();
        }
        if (!m_inclinePC.isEnable()) {
            m_inclinePC.enable();
        }
        m_inclinePC.setSetpoint(target); /// 39);
        SmartDashboard.putNumber("Error", m_inclinePC.getError());
        SmartDashboard.putNumber("PID Value:", m_inclinePC.get());
        if (!m_inclinePC.onTarget()) {
            inclineChecked(-m_inclinePC.get());
        } else {
            inclineChecked(0);
        }
    }

    public void load() {
        m_Reloader.set(DoubleSolenoid.Value.kForward);
    }

    public void reload() {
        m_Reloader.set(DoubleSolenoid.Value.kReverse);
    }

    public void shoot() {
        if (!fastMode) {
            shoot(0.80, 0.85);
        } else {
            shoot(0.95, 1);
        }
    }

    public void stop() {
        shoot(0, 0);
    }

    public void disableInclinePID() {
        m_inclinePC.disable();
    }

    public void shoot(double backSpeed, double frontSpeed) {
        m_VFDBack.set(-backSpeed);
        m_VFDFront.set(-frontSpeed);
        
    }

    public void turn(double speed) {
        turnChecked(speed * 0.5);
    }
    public void turnChecked(double speed) {
        SmartDashboard.putNumber("Turret speed", speed);
    m_Turret.set(speed);
    }

    public void turnCentre() {
        turnPID(1.314);
    }
//1.353
//1.273
    public void turnPID(double target) {
        if (m_turnPC.getSetpoint() != target) {
            m_turnPC.reset();
        }
        if (!m_turnPC.isEnable()) {
            m_turnPC.enable();
        }
        m_turnPC.setSetpoint(target);
        SmartDashboard.putNumber("Turn error", m_turnPC.getError());
        SmartDashboard.putNumber("Turn PID Value:", m_turnPC.get());
        if (!m_turnPC.onTarget()){
            turnChecked((-m_turnPC.get()) * (//1.8
                    1.75
                    / (Math.abs(m_turnPC.getError()))));
        } else {
            turn(0);
        }
    }
}
