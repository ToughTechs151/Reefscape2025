package frc.sim;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ClawConstants;

/** Constants utility class for the claw simulation. */
public final class Constants {

  private Constants() {
    throw new IllegalStateException("Utility class");
  }

  /** Claw simulation constants. */
  public static final class ClawSim {
    private ClawSim() {
      throw new IllegalStateException("ClawSim Utility Class");
    }

    public static final double CLAW_MASS_KG = 3.5;
    public static final double CLAW_LENGTH_INCHES = 12;
    public static final double CLAW_LENGTH_METERS = Units.inchesToMeters(CLAW_LENGTH_INCHES);
    public static final double START_ANGLE_RADS = ClawConstants.MIN_ANGLE_RADS;
    public static final int ENCODER_PRR =
        4096; // Only used to simulate noise in position measurement
    public static final double ENCODER_DISTANCE_PER_PULSE =
        2.0 * Math.PI / ENCODER_PRR / ClawConstants.GEAR_RATIO;
  }

  /** Elevator simulation constants. */
  public static final class ElevatorSimConstants {
    private ElevatorSimConstants() {
      throw new IllegalStateException("ElevatorSimConstants Utility Class");
    }

    // The effective load lifted by the elevator. For a continuous elevator this is the total
    // moving mass. For a two stage cascade this is the mass of the first moving stage plus
    // two times the mass of the carriage.
    public static final double EFFECTIVE_MASS = 10.0; // kg
  }

  /** Drivetrain simulation constants. */
  public static final class DriveSimConstants {
    private DriveSimConstants() {
      throw new IllegalStateException("DriveSim Utility Class");
    }

    public static final int NUM_MOTORS = 2;
    public static final double KV_LINEAR = 2.0;
    public static final double KA_LINEAR = 0.2;
    public static final double KV_ANGULAR = 3.0;
    public static final double KA_ANGULAR = 0.3;
    // Gain to apply to voltage command to get realistic current (0-1)
    public static final double VOLT_SCALE_FACTOR = 0.7;
  }

  /** Launcher simulation constants. */
  public static final class RollerSimConstants {
    private RollerSimConstants() {
      throw new IllegalStateException("RollerSimConstants Utility Class");
    }

    public static final double ROLLER_MOI_KG_METERS2 = 0.05;
  }

  /** Launcher simulation constants. */
  public static final class LauncherSimConstants {
    private LauncherSimConstants() {
      throw new IllegalStateException("LauncherSimConstants Utility Class");
    }

    public static final double LAUNCHER_MOI_KG_METERS2 = 0.001;
  }
}
