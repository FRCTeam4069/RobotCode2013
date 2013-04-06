package frc.t4069.year2.robots;

public class Autonomous {
/*	private String[] parameters;
	private boolean[] completed;
	public static final String SHOOTER_SPEED = "SHOOTER_SPEED",
			HORIZ_ANGLE = "HORIZ_ANGLE", VERT_ANGLE = "VERT_ANGLE",
			SPIN_UP = "SPIN_UP", FORWARD_SOLENOID = "FORWARD_SOLENOID",
			BACKWARD_SOLENOID = "BACKWARD_SOLENOID", COMPRESS = "COMPRESS";

	public Autonomous(String[] params) {
		parameters = params;
		completed = new boolean[params.length];
	}

	public void doCommand() {
		int index = -1;
		for (int i = 0; i < completed.length; i++) {
			if (!completed[i]) {
				index = i;
				break;
			}
			if (i == completed.length - 1)
				return;
		}
		String s;
		s = parameters[index];
		completed[index] = execParameter(s);
	}

	private boolean execParameter(String list) {
		boolean finished = true;
		String param;
		for (int i = 0; i < list.length(); param = list.split(",")[i], i++) {
			switch (param.split("=")[0]) {
			case SHOOTER_SPEED:
				// dummy, no encoders
				break;
			case SPIN_UP:

			case VERT_ANGLE:
				double angle = Double.parseDouble(param
						.split("=")[1]);
				The2013Robot.shooter.setInclineTarget(angle);
				The2013Robot.shooter.inclinePID();
				finished = Math.abs(1-(The2013Robot.shooter.getIncline() / angle)) < 0.09;
				break;
			case HORIZ_ANGLE:
				break;
			}
		}
		return finished;
	}
*/}
