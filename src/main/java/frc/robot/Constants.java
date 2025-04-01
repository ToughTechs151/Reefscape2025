// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import java.util.List;
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
  public static final boolean SD_SHOW_ROLLER_EXTENDED_LOGGING_DATA = true;
  public static final boolean SD_SHOW_DRIVE_EXTENDED_LOGGING_DATA = true;

  public static final boolean LOOP_TIMING_LOG = false;

  // Set to true to enable using Tunable Numbers
  public static final boolean TUNING_MODE = true;

  // Set to true to log each frame of command execution. To false to disable.
  public static final boolean COMMAND_EXECUTE_LOG = false;

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
    public static final double CLAW_MAX_VELOCITY_RAD_PER_SEC = Units.degreesToRadians(180.0);
    public static final double CLAW_MAX_ACCELERATION_RAD_PER_SEC2 = Units.degreesToRadians(540.0);

    public static final double GEAR_RATIO = 75 * 30 / 12.0;
    public static final double CLAW_RAD_PER_ENCODER_ROTATION = 2.0 * Math.PI / GEAR_RATIO;
    public static final double RPM_TO_RAD_PER_SEC = CLAW_RAD_PER_ENCODER_ROTATION / 60;

    // Claw positions.  Horizontal = 0 radians. Assume claw starts at lowest (rest) position
    public static final double CLAW_LEVEL1_RADS = Units.degreesToRadians(18.0);
    public static final double CLAW_LEVEL2_AND_LEVEL3_RADS = Units.degreesToRadians(42.0);
    public static final double CLAW_SAFE_ANGLE_RADS = Units.degreesToRadians(45.0);
    public static final double CLAW_LEVEL4_RADS = Units.degreesToRadians(80.0);
    public static final double CLAW_ALGAE_RADS = Units.degreesToRadians(178.0);
    public static final double CLAW_PROCESSOR_RADS = Units.degreesToRadians(178.0);
    public static final double CLAW_OFFSET_RADS = Units.degreesToRadians(18.0);
    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(18.0);
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(180.0);
    public static final double POS_INCREMENT = Units.degreesToRadians(1.0); // For small adjustments
    public static final double POSITION_TOLERANCE = Units.degreesToRadians(4.0);
    public static final double VELOCITY_TOLERANCE = Units.degreesToRadians(10.0);
    public static final double ABSOLUTE_OFFSET_DEGREES = 222.6;
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
    public static final double ELEVATOR_KP = 24.0;
    public static final double ELEVATOR_KS = 0;
    public static final double ELEVATOR_KG = 0.7;
    public static final double ELEVATOR_KV_VOLTS_PER_METER_PER_SEC = 5.5;
    public static final double ELEVATOR_MAX_VELOCITY_METERS_PER_SEC = 1.3;
    public static final double ELEVATOR_MAX_ACCELERATION_METERS_PER_SEC2 = 3.6;

    // Spool Diameter in Inches
    public static final double SPOOL_DIAMETER = Units.inchesToMeters(1.73);

    public static final double GEAR_RATIO = 15.0;

    // Factor of 2 is due to using a cascade elevator
    public static final double ELEVATOR_METERS_PER_ENCODER_ROTATION =
        2 * SPOOL_DIAMETER * Math.PI / GEAR_RATIO;

    public static final double RPM_TO_METERS_PER_SEC = ELEVATOR_METERS_PER_ENCODER_ROTATION / 60;
    public static final double ELEVATOR_LOAD_CORAL = Units.inchesToMeters(0.0);
    public static final double ELEVATOR_LEVEL1 = Units.inchesToMeters(1.0);
    public static final double ELEVATOR_LEVEL2 = Units.inchesToMeters(11.5);
    public static final double ELEVATOR_LEVEL3 = Units.inchesToMeters(26.5);
    public static final double ELEVATOR_LEVEL4 = Units.inchesToMeters(55.0);
    public static final double ELEVATOR_LEVEL2_ALGAE = Units.inchesToMeters(18.0);
    public static final double ELEVATOR_LEVEL3_ALGAE = Units.inchesToMeters(33.9);
    public static final double ELEVATOR_PROCESSOR = Units.inchesToMeters(2.0);
    public static final double ELEVATOR_OFFSET_METERS = 0.0;

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double ELEVATOR_MIN_HEIGHT_METERS = 0.0;
    public static final double ELEVATOR_MAX_HEIGHT_METERS = Units.inchesToMeters(63);

    public static final double POSITION_TOLERANCE_METERS = 0.04;
    public static final double VELOCITY_TOLERANCE_METERS = 0.01;

    public static final double POS_INCREMENT = Units.inchesToMeters(1.0); // For small adjustments
  }

  /** Constants used for the Roller subsystem. */
  public static final class RollerConstants {

    private RollerConstants() {
      throw new IllegalStateException("RollerConstants Utility Class");
    }

    public static final int ROLLER_MOTOR_PORT = 17;
    public static final int CURRENT_LIMIT = 40;

    // Constants tunable through TunableNumbers
    public static final double POSITION_KP = 0.5;
    public static final double SPEED_KP = 0.00075;
    public static final double SPEED_KS_VOLTS = 0.0;
    public static final double SPEED_KV_VOLTS_PER_RPM = 0.0055;
    public static final double ROLLER_SET_POINT_FORWARD_RPM = 250.0;
    public static final double ROLLER_SET_POINT_REVERSE_RPM = -500.0;
    public static final double ROLLER_LOAD_CORAL_RPM = -500.0;

    public static final double ROLLER_GEAR_RATIO =
        30.0 / 12.0; // Ratio of motor rotations to output rotations
    public static final double ROLLER_ROTATIONS_PER_ENCODER_ROTATION = 1.0 / ROLLER_GEAR_RATIO;
    public static final double ROLLER_TOLERANCE_RPM = 20;

    public static final int CANRANGE_PORT = 19;
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

  /** Constants used for the swerve drive subsystem. */
  public static final class DriveConstants {

    private DriveConstants() {
      throw new IllegalStateException("DriveConstants Utility Class");
    }

    public static final double ROBOT_MASS = 40.0 * 0.453592; // 40lbs * kg per pound
    public static final Matter CHASSIS =
        new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms spark max velocity lag
    // Speed for Neo Vortex at 6700 RPM, 6.75:1 gears and 4" wheels
    public static final double MAX_SPEED = Units.feetToMeters(6700 / 6.75 / 60 * 4 * Math.PI / 12);

    public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.45, 4);
    public static final Translation2d RED_REEF_CENTER = new Translation2d(12.95, 4);

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static final double POV_SPEED = 0.05;

    public static final Boolean ENABLE_VISION = true;
    public static final double MAX_TAG_DISTANCE = 3.0; // meters
    public static final double MAX_POSE_AMBIGUITY = 0.1;

    public static final Boolean USE_ALLIANCE = true;

    public static final Pose2d BLUE_START_POSE =
        new Pose2d(new Translation2d(Meter.of(7.5), Meter.of(1.9)), Rotation2d.fromDegrees(180));
    public static final Pose2d RED_START_POSE =
        new Pose2d(new Translation2d(Meter.of(9.9), Meter.of(6.1)), Rotation2d.fromDegrees(0));

    // Constants for drive to pose initial path following
    public static final PathConstraints DRIVE_POSE_CONSTRAINTS =
        new PathConstraints(1.0, 4.0, Units.degreesToRadians(180), Units.degreesToRadians(720));

    public static final double DISTANCE_UNTIL_PID = Units.inchesToMeters(3);
    public static final double ROTATION_GOAL_BEFORE_PID = 1;
    public static final LinearVelocity PATH_FIND_END_VELOCITY = MetersPerSecond.of(0.0);

    // Constants for drive to pose final Holonomic controller
    public static final Rotation2d ROTATION_TOLERANCE = Rotation2d.fromDegrees(2.0);
    public static final Distance POSITION_TOLERANCE = Centimeter.of(1.0);
    public static final LinearVelocity SPEED_TOLERANCE = InchesPerSecond.of(1);
    public static final Time END_TRIGGER_DEBOUNCE = Seconds.of(0.1);
    public static final Time AUTO_ALIGN_ADJUST_TIMEOUT = Seconds.of(1.0);
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants ROTATION_PID = new PIDConstants(5.0, 0.0, 0.0);
  }

  /** Constants used for positions on the field. */
  public static class FieldConstants {
    private FieldConstants() {
      throw new IllegalStateException("FieldConstants Utility Class");
    }

    // Positions of the April Tags on the reef
    public static final List<Pose2d> REEF_POSITIONS =
        List.of(
            new Pose2d(
                Units.inchesToMeters(530.49), // 6
                Units.inchesToMeters(129.97),
                Rotation2d.fromDegrees(-60)),
            new Pose2d(
                Units.inchesToMeters(546.87), // 7
                Units.inchesToMeters(158.30),
                Rotation2d.fromDegrees(0)),
            new Pose2d(
                Units.inchesToMeters(530.49), // 8
                Units.inchesToMeters(186.83),
                Rotation2d.fromDegrees(60)),
            new Pose2d(
                Units.inchesToMeters(497.77), // 9
                Units.inchesToMeters(186.83),
                Rotation2d.fromDegrees(120)),
            new Pose2d(
                Units.inchesToMeters(481.39), // 10
                Units.inchesToMeters(158.30),
                Rotation2d.fromDegrees(180)),
            new Pose2d(
                Units.inchesToMeters(497.77), // 11
                Units.inchesToMeters(129.97),
                Rotation2d.fromDegrees(-120)),
            new Pose2d(
                Units.inchesToMeters(160.39), // 17
                Units.inchesToMeters(129.97),
                Rotation2d.fromDegrees(-120)),
            new Pose2d(
                Units.inchesToMeters(144.00), // 18
                Units.inchesToMeters(158.30),
                Rotation2d.fromDegrees(180)),
            new Pose2d(
                Units.inchesToMeters(160.39), // 19
                Units.inchesToMeters(186.63),
                Rotation2d.fromDegrees(120)),
            new Pose2d(
                Units.inchesToMeters(193.10), // 20
                Units.inchesToMeters(186.83),
                Rotation2d.fromDegrees(60)),
            new Pose2d(
                Units.inchesToMeters(209.49), // 21
                Units.inchesToMeters(158.30),
                Rotation2d.fromDegrees(0)),
            new Pose2d(
                Units.inchesToMeters(193.10), // 22
                Units.inchesToMeters(129.97),
                Rotation2d.fromDegrees(-60)));

    // Offsets for the robot to the left and right of the reef April Tags
    public static final double REEF_FORWARD_OFFSET = Units.inchesToMeters(16.0);
    public static final double REEF_RIGHT_OFFSET = Units.inchesToMeters((13 / 2) - 0.375);
    public static final double REEF_LEFT_OFFSET = Units.inchesToMeters((-13 / 2) - 0.375);
    public static final Translation2d REEF_SHIFT_LEFT =
        new Translation2d(REEF_FORWARD_OFFSET, REEF_LEFT_OFFSET);
    public static final Translation2d REEF_SHIFT_RIGHT =
        new Translation2d(REEF_FORWARD_OFFSET, REEF_RIGHT_OFFSET);
  }
}
