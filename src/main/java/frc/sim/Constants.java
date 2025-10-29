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

    /** Mass of the claw arm in kilograms. */
    public static final double CLAW_MASS_KG = 3.5;
    /** Length of the claw arm in inches. */
    public static final double CLAW_LENGTH_INCHES = 12;
    /** Length of the claw arm in meters. */
    public static final double CLAW_LENGTH_METERS = Units.inchesToMeters(CLAW_LENGTH_INCHES);
    /** Starting angle for the claw simulation in radians. */
    public static final double START_ANGLE_RADS = ClawConstants.MIN_ANGLE_RADS;
    /** Encoder pulses per revolution - only used to simulate noise in position measurement. */
    public static final int ENCODER_PRR =
        4096; // Only used to simulate noise in position measurement
    /** Distance per encoder pulse for simulation - accounts for gear ratio. */
    public static final double ENCODER_DISTANCE_PER_PULSE =
        2.0 * Math.PI / ENCODER_PRR / ClawConstants.GEAR_RATIO;
  }

  /** Elevator simulation constants. */
  public static final class ElevatorSimConstants {
    private ElevatorSimConstants() {
      throw new IllegalStateException("ElevatorSimConstants Utility Class");
    }

    /** The effective load lifted by the elevator. For a continuous elevator this is the total
     * moving mass. For a two stage cascade this is the mass of the first moving stage plus
     * two times the mass of the carriage. */
    public static final double EFFECTIVE_MASS = 5.0; // kg
  }

  /** Drivetrain simulation constants. */
  public static final class DriveSimConstants {
    private DriveSimConstants() {
      throw new IllegalStateException("DriveSim Utility Class");
    }

    /** Number of motors driving each side of the drivetrain. */
    public static final int NUM_MOTORS = 2;
    /** Linear velocity feedforward velocity gain (V/(m/s)). */
    public static final double KV_LINEAR = 2.0;
    /** Linear velocity feedforward acceleration gain (V/(m/s²)). */
    public static final double KA_LINEAR = 0.2;
    /** Angular velocity feedforward velocity gain (V/(rad/s)). */
    public static final double KV_ANGULAR = 3.0;
    /** Angular velocity feedforward acceleration gain (V/(rad/s²)). */
    public static final double KA_ANGULAR = 0.3;
    /** Gain to apply to voltage command to get realistic current (0-1). */
    public static final double VOLT_SCALE_FACTOR = 0.7;
  }

  /** Roller simulation constants. */
  public static final class RollerSimConstants {
    private RollerSimConstants() {
      throw new IllegalStateException("RollerSimConstants Utility Class");
    }

    /** Moment of inertia for the roller mechanism in kg·m². */
    public static final double ROLLER_MOI_KG_METERS2 = 0.01;
  }

  /** Launcher simulation constants. */
  public static final class LauncherSimConstants {
    private LauncherSimConstants() {
      throw new IllegalStateException("LauncherSimConstants Utility Class");
    }

    /** Moment of inertia for the launcher mechanism in kg·m². */
    public static final double LAUNCHER_MOI_KG_METERS2 = 0.001;
  }
}
