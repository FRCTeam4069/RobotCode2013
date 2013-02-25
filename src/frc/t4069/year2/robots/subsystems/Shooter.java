package frc.t4069.year2.robots.subsystems;

import java.util.Date;
import edu.wpi.first.wpilibj.*;
import frc.t4069.year2.robots.Constants;
import frc.t4069.year2.utils.Potentiometer;
import frc.t4069.year2.utils.math.LowPassFilter;

public class Shooter {
	class AngleOutput implements PIDOutput {
		public double value = 0;

		@Override
		public void pidWrite(double output) {
			value = output;
		}

	}

	class AnglePotentiometer implements PIDSource {
		private Potentiometer m_potentiometer;
		private LowPassFilter m_lpf = new LowPassFilter(25);

		public AnglePotentiometer(Potentiometer pot) {
			m_potentiometer = pot;
		}

		@Override
		public double pidGet() {
			return m_lpf.calculate(m_potentiometer.getAngle(true));
		}

	}

	protected SpeedController m_VFDBack;
	protected SpeedController m_VFDFront;
	protected LowPassFilter m_backLPF, m_frontLPF;
	protected SpeedController m_Turret, m_Incliner;
	protected DoubleSolenoid m_Reloader;
	protected Compressor m_Compressor;
	protected Potentiometer m_TurningPot, m_RisingPot;
	private PIDController m_turnpc;
	private PIDController m_inclinepc;
	private PIDSource m_inclinepidsource;
	private PIDOutput m_inclinepidoutput;
	private double inclined = 0.06, inclinei = 0.01, inclinep = 0.24;
	private double turnd = 0.06, turni = 0.01, turnp = 0.24;
	private PIDSource m_turnpidsource;
	private PIDOutput m_turnpidoutput;

	private LowPassFilter m_InclineLPF = new LowPassFilter(50);

	private LowPassFilter m_TurnLPF = new LowPassFilter(10);

	public Shooter() {
		this(150);
	}

	public Shooter(double RC) {
		this(new Jaguar(Constants.FRONT_SHOOTER), new Jaguar(
				Constants.BACK_SHOOTER), new Jaguar(Constants.INCLINER_VFD),
				new Jaguar(Constants.TURN_VFD), new DoubleSolenoid(
						Constants.RELOADER_SOLENOID_FORWARD,
						Constants.RELOADER_SOLENOID_BACKWARD), new Compressor(
						Constants.PRESSURE_SWITCH, Constants.COMPRESSOR),
				Constants.TURN_POT, Constants.INCLINER_POT, RC);
	}

	/**
	 * Creates a new Shooter object. Note: The use of Talons for shooter control
	 * is recommended.
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
		m_TurningPot = new Potentiometer(turnPot, -180, 72);
		m_RisingPot = new Potentiometer(risePot, -263.150, 92.0444649);
		m_turnpidsource = new AnglePotentiometer(m_TurningPot);
		m_turnpidoutput = new AngleOutput();
		m_inclinepidsource = new AnglePotentiometer(m_RisingPot);
		m_inclinepidoutput = new AngleOutput();
		m_backLPF = new LowPassFilter(RC);
		m_frontLPF = new LowPassFilter(RC);
		m_turnpc = new PIDController(turnp, turni, turnd, m_turnpidsource,
				m_turnpidoutput);
		m_turnpc.setTolerance(1.5);
		m_turnpc.setOutputRange(-1, 1);
		m_inclinepc = new PIDController(inclinep, inclinei, inclined,
				m_inclinepidsource, m_inclinepidoutput);
		m_inclinepc.setOutputRange(-1, 1);
		m_inclinepc.setTolerance(1.5);
		m_Compressor.start();
	}

	public void compress(boolean enable) {
		if (enable)
			m_Compressor.start();
		else
			m_Compressor.stop();
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

	public double getTurn() {
		return m_TurningPot.getAngle(true);
	}

	public double getTurn(boolean degrees) {
		return m_TurningPot.getAngle(degrees);
	}

	public void incline(double speed) {
		m_Incliner.set(m_InclineLPF.calculate(speed * 0.2));
	}

	public void incline(double angle, boolean degrees) {
		if (Math.abs(1 - angle / m_RisingPot.getAngle(degrees)) < 0.05) {
			m_inclinepc.disable();
			return;
		}

	}

	public void inclinePID() {
		if (!m_inclinepc.isEnable()) {
			m_inclinepc.enable();
		}
		m_Incliner.set(m_inclinepc.get());
	}

	public void load() {
		m_Reloader.set(DoubleSolenoid.Value.kForward);
	}

	public void reload() {
		m_Reloader.set(DoubleSolenoid.Value.kReverse);
	}

	public void setInclineTarget(double angle) {
		m_inclinepc.setSetpoint(angle);
		if (!m_inclinepc.isEnable())
			m_inclinepc.enable();
	}

	public void shoot() {
		shoot(1, 1);
	}

	public void stop() {
		shoot(0, 0);
	}

	public void disableInclinePID() {
		m_inclinepc.disable();
	}

	public void shoot(double backSpeed, double frontSpeed) {
		m_VFDBack.set(-m_backLPF.calculate(backSpeed));
		m_VFDFront.set(-m_frontLPF.calculate(frontSpeed));
	}

	public void turn(double speed) {
		m_Turret.set(speed * 0.3);
	}

}
