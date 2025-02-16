// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private Constants() {
    throw new IllegalStateException("Utility class");
  }

  // Run time options

  // Set to true to log Joystick data. To false otherwise.
  public static final boolean LOG_JOYSTICK_DATA = true;

  // Set to true to send telemetry data to Live Window. To false
  // to disable it.
  public static final boolean LW_TELEMETRY_ENABLE = false;

  // Set a Global Constant to either Show or Hide extended logging data for each of the 5 subsystems
  // Set to true to show extended logging data.
  // Set to false to hide extended logging data.
  public static final boolean SD_SHOW_CLAW_EXTENDED_LOGGING_DATA = true;
  public static final boolean SD_SHOW_LAUNCHER_EXTENDED_LOGGING_DATA = true;
  public static final boolean SD_SHOW_CLIMBER_EXTENDED_LOGGING_DATA = true;
  public static final boolean SD_SHOW_INTAKE_EXTENDED_LOGGING_DATA = true;
  public static final boolean SD_SHOW_DRIVE_EXTENDED_LOGGING_DATA = true;

  public static final boolean LOOP_TIMING_LOG = false;

  // Set to true to enable using Tunable Numbers
  public static final boolean TUNING_MODE = true;

  // Set to true to log each frame of command execution. To false to disable.
  public static final boolean COMMAND_EXECUTE_LOG = false;

  // Camera ID
  public static final int CAMERA_0 = 0;
  public static final int CAMERA_1 = 1;

  /** Constants used for the Claw subsystem. */
  public static final class ClawConstants {

    private ClawConstants() {
      throw new IllegalStateException("ClawConstants Utility Class");
    }

    public static final int MOTOR_PORT = 16;
    public static final int CURRENT_LIMIT = 40;
    public static final boolean INVERTED = true;

    // Constants tunable through TunableNumbers
    public static final double CLAW_KP = 6.0;
    public static final double CLAW_KS = 0.0;
    public static final double CLAW_KG = 0.1;
    public static final double CLAW_KV_VOLTS_PER_RAD_PER_SEC = 3.5;
    public static final double CLAW_MAX_VELOCITY_RAD_PER_SEC = Units.degreesToRadians(90);
    public static final double CLAW_MAX_ACCELERATION_RAD_PER_SEC2 = Units.degreesToRadians(360);

    public static final double GEAR_RATIO = 75 * 30 / 12.0;
    public static final double CLAW_RAD_PER_ENCODER_ROTATION = 2.0 * Math.PI / GEAR_RATIO;
    public static final double RPM_TO_RAD_PER_SEC = CLAW_RAD_PER_ENCODER_ROTATION / 60;

    // Claw positions.  Horizontal = 0 radians. Assume claw starts at lowest (rest) position
    public static final double CLAW_LEVEL1_RADS = Units.degreesToRadians(18.0);
    public static final double CLAW_LEVEL2_AND_LEVEL3_RADS = Units.degreesToRadians(50.0);
    public static final double CLAW_LEVEL4_RADS = Units.degreesToRadians(80.0);
    public static final double CLAW_ALGAE_RADS = Units.degreesToRadians(173.0);
    public static final double CLAW_OFFSET_RADS = Units.degreesToRadians(18.0);
    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(18.0);
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(196.0);
    public static final double POS_INCREMENT = Units.degreesToRadians(2.0); // For small adjustments
    public static final double POSITION_TOLERANCE = Units.degreesToRadians(4.0);
    public static final double VELOCITY_TOLERANCE = Units.degreesToRadians(10.0);
  }

  /** Constants used for the Elevator subsystem. */
  public static final class ElevatorConstants {

    private ElevatorConstants() {
      throw new IllegalStateException("ElevatorConstants Utility Class");
    }

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final int MOTOR_PORT = 18;
    public static final int CURRENT_LIMIT = 40;
    public static final boolean INVERTED = true;
    // Constants tunable through TunableNumbers
    public static final double ELEVATOR_KP = 12.0;
    public static final double ELEVATOR_KS = 0.54;
    public static final double ELEVATOR_KG = 0.7;
    public static final double ELEVATOR_KV_VOLTS_PER_METER_PER_SEC = 6.25;
    public static final double ELEVATOR_MAX_VELOCITY_METERS_PER_SEC = 0.15;
    public static final double ELEVATOR_MAX_ACCELERATION_METERS_PER_SEC2 = 1.2;

    // Spool Diameter in Inches
    public static final double SPOOL_DIAMETER = Units.inchesToMeters(1.73);

    public static final double GEAR_RATIO = 16.0;

    // Factor of 2 is due to using a cascade elevator
    public static final double ELEVATOR_METERS_PER_ENCODER_ROTATION =
        2 * SPOOL_DIAMETER * Math.PI / GEAR_RATIO;

    public static final double RPM_TO_METERS_PER_SEC = ELEVATOR_METERS_PER_ENCODER_ROTATION / 60;
    public static final double ELEVATOR_LEVEL1 = Units.inchesToMeters(0);
    public static final double ELEVATOR_LEVEL2 = Units.inchesToMeters(11.5);
    public static final double ELEVATOR_LEVEL3 = Units.inchesToMeters(22.5);
    public static final double ELEVATOR_LEVEL4 = Units.inchesToMeters(52);
    public static final double ELEVATOR_OFFSET_METERS = 0.0;

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double ELEVATOR_MIN_HEIGHT_METERS = 0.0;
    public static final double ELEVATOR_MAX_HEIGHT_METERS = Units.inchesToMeters(63);

    public static final double POSITION_TOLERANCE_METERS = 0.03;
    public static final double VELOCITY_TOLERANCE_METERS = 0.01;
  }

  /** Constants used for the Launcher subsystem. */
  public static final class IntakeConstants {

    private IntakeConstants() {
      throw new IllegalStateException("IntakeConstants Utility Class");
    }

    public static final int INTAKE_MOTOR_PORT = 17;
    public static final int CURRENT_LIMIT = 40;

    // Constants tunable through TunableNumbers
    public static final double INTAKE_KP = 6.0;
    public static final double INTAKE_KS_VOLTS = 0.2;
    public static final double INTAKE_KV_VOLTS_PER_RPM = 1.8;
    public static final double INTAKE_SET_POINT_FORWARD_RPM = 300.0;
    public static final double INTAKE_SET_POINT_REVERSE_RPM = -400.0;
    public static final double INTAKE_SPEED_THRESHOLD_RPM = 250.0;
    public static final double INTAKE_CURRENT_THRESHOLD_AMPS = 8.0;

    public static final double INTAKE_GEAR_RATIO =
        12.0; // Ratio of motor rotations to output rotations
    public static final double INTAKE_ROTATIONS_PER_ENCODER_ROTATION = 1.0 / INTAKE_GEAR_RATIO;
    public static final double INTAKE_TOLERANCE_RPM = 20;
  }

  /** Constants used for assigning operator input. */
  public static final class OIConstants {

    private OIConstants() {
      throw new IllegalStateException("OIConstants Utility Class");
    }

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class DriveConstants {
    public static final double ROBOT_MASS = 40.0 * 0.453592; // 40lbs * kg per pound
    public static final Matter CHASSIS =
        new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms spark max velocity lag
    // Speed for Neo Vortex at 6700 RPM, 6.75:1 gears and 4" wheels
    public static final double MAX_SPEED = Units.feetToMeters(6700 / 6.75 / 60 * 4 * Math.PI / 12);

    public static final double POV_SPEED = 0.1;

    public static final Boolean ENABLE_VISION = true;
    public static final Boolean USE_ALLIANCE = false;
  }
}
