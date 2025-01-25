// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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
  public static final boolean SD_SHOW_ARM_EXTENDED_LOGGING_DATA = true;
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

  /** Constants used for the Arm subsystem. */
  public static final class ArmConstants {

    private ArmConstants() {
      throw new IllegalStateException("ArmConstants Utility Class");
    }

    public static final int MOTOR_PORT = 6;
    public static final int BEAM_BREAKER_PORT = 1;
    public static final int CURRENT_LIMIT = 40;

    // Constants tunable through TunableNumbers
    public static final double ARM_KP = 6.0;
    public static final double ARM_KS = 0.2;
    public static final double ARM_KG = 0.3;
    public static final double ARM_KV_VOLTS_PER_RAD_PER_SEC = 1.8;
    public static final double ARM_MAX_VELOCITY_RAD_PER_SEC = 240;
    public static final double ARM_MAX_ACCELERATION_RAD_PER_SEC2 = 720;

    public static final double GEAR_RATIO = 100;
    public static final double ARM_RAD_PER_ENCODER_ROTATION = 2.0 * Math.PI / GEAR_RATIO;
    public static final double RPM_TO_RAD_PER_SEC = ARM_RAD_PER_ENCODER_ROTATION / 60;

    // Arm positions.  Horizontal = 0 radians. Assume arm starts at lowest (rest) position
    public static final double ARM_FORWARD_POSITION_RADS = Units.degreesToRadians(-7.0);
    public static final double ARM_BACK_POSITION_RADS = Units.degreesToRadians(170.0);
    public static final double ARM_OFFSET_RADS = Units.degreesToRadians(180.0);
    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(-18.0);
    public static final double MAX_ANGLE_RADS = ARM_OFFSET_RADS;
    public static final double POS_INCREMENT = Units.degreesToRadians(2.0); // For small adjustments
    public static final double POSITION_TOLERANCE = Units.degreesToRadians(4.0);
    public static final double VELOCITY_TOLERANCE = Units.degreesToRadians(10.0);
  }

  /** Constants used for the Launcher subsystem. */
  public static final class IntakeConstants {

    private IntakeConstants() {
      throw new IllegalStateException("IntakeConstants Utility Class");
    }

    public static final int INTAKE_MOTOR_PORT = 7;
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
  }

  public static final class DriveConstants {
    public static final double ROBOT_MASS = 40.0 * 0.453592; // 40lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms spark max velocity lag
    // Speed for Neo Vortex at 6700 RPM, 6.75:1 gears and 4" wheels
    public static final double MAX_SPEED  = Units.feetToMeters(6700/6.75/60*4*Math.PI/12);
    public static final double SPEED_SCALING = 0.5; // Scale joystick inputs to limit speed
    public static final double SPEED_SCALING_3 = Math.pow(SPEED_SCALING, 1/3.0); // Scale for inputs^3
  
    public static final Boolean ENABLE_VISION  = true;
  }
}
