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

    class Output implements PIDOutput {

        public double value = 0;

        public void pidWrite(double output) {
            value = output;
        }
    }

    class RPMEncoder implements PIDSource {

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

    class AnglePotentiometer implements PIDSource {

        private Potentiometer m_potentiometer;
        //  private LowPassFilter m_lpf = new LowPassFilter(5);

        public AnglePotentiometer(Potentiometer pot) {
            m_potentiometer = pot;
        }

        public double pidGet() {
            return (m_potentiometer.getAngle(true));/// 39);
        }
    }
    protected SpeedController m_VFDBack;
    protected SpeedController m_VFDFront;
    protected LowPassFilter m_backLPF, m_frontLPF;
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
    private PIDSource m_inclinepidsource;
    private PIDOutput m_inclinepidoutput;
    private double inclined = 0,//0 initial constant
            inclinei = 0.0005//0.0009 //0.005 initial constant 
            , inclinep = 0.075; //0.15 initial constant
    private LowPassFilter m_InclineLPF = new LowPassFilter(50);
    private LowPassFilter m_TurnLPF = new LowPassFilter(10);

    public Shooter() {
        this(90);
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
        m_TurningPot = new Potentiometer(turnPot, -180, 72); // magic
        m_RisingPot = new Potentiometer(risePot, -263.150, 92.0444649); // is friendship
        m_inclinepidsource = new AnglePotentiometer(m_RisingPot);
        m_inclinepidoutput = new Output();
        m_inclinePC = new PIDController(inclinep, inclinei, inclined,
                m_inclinepidsource, m_inclinepidoutput, 0.025);
        m_inclinePC.setOutputRange(-0.5, 0.5);
        m_inclinePC.setInputRange(0, 39);
        //m_inclinePC.setTolerance(1.5);
        m_inclinePC.setAbsoluteTolerance(0.08);
        m_Compressor.start();
    }

    public void compress(boolean enable) {
        if (enable) {
            m_Compressor.start();
        } else {
            m_Compressor.stop();
        }
    }

    public double getIncline() {
        return m_RisingPot.getAngle(true);
    }

    public double getIncline(boolean degrees) {
        return m_RisingPot.getAngle(degrees);
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

        if (!m_inclinePC.isEnable()) {
            m_inclinePC.enable();
        }
        m_inclinePC.setSetpoint(target); /// 39);
        SmartDashboard.putNumber("Error", m_inclinePC.getError());
        SmartDashboard.putNumber("PID Value:", m_inclinePC.get());
        inclineChecked(-m_inclinePC.get());
    }

    public void load() {
        m_Reloader.set(DoubleSolenoid.Value.kForward);
    }

    public void reload() {
        m_Reloader.set(DoubleSolenoid.Value.kReverse);
    }

    public void setInclineTarget(double angle) {
        m_inclinePC.setSetpoint(angle);
        if (!m_inclinePC.isEnable()) {
            m_inclinePC.enable();
        }
    }
    public boolean fastMode = false;

    public void shoot() {
        if (!fastMode) {
            shoot(0.85, 0.90);
        } else {
            shoot(0.9, 1);
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
        m_Turret.set(speed * 0.5);
    }
}
