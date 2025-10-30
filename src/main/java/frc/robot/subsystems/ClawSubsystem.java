// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.util.TunableNumber;

/**
 * /** Subsystem that controls the claw mechanism using a Profiled PID Controller for position
 * control.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of ClawSubsystem
 * CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
 * ClawSubsystem clawSubsystem = new ClawSubsystem(motor);
 *
 * // Move the claw to a specific position
 * Command moveToPositionCommand = clawSubsystem.moveToPosition(90.0);
 * moveToPositionCommand.schedule();
 *
 * // Hold the claw at the current position
 * Command holdPositionCommand = clawSubsystem.holdPosition();
 * holdPositionCommand.schedule();
 *
 * // Shift the claw's position up by a fixed increment
 * Command shiftUpCommand = clawSubsystem.shiftUp();
 * shiftUpCommand.schedule();
 *
 * // Shift the claw's position down by a fixed increment
 * Command shiftDownCommand = clawSubsystem.shiftDown();
 * shiftDownCommand.schedule();
 * }
 *
 * Code Analysis:
 * - Main functionalities:
 *   - Control the movement of an claw using a Profiled PID Controller
 *   - Move the claw to a specific position
 *   - Hold the claw at the current position
 *   - Shift the claw's position up or down by a fixed increment
 * - Methods:
 *   - {@code periodic()}: Published telemetry with information about the claw's state.
 *   - {@code useOutput()}: Generates the motor command using the PID controller and feedforward.
 *   - {@code moveToPosition(double goal)}: Returns a Command that moves the claw to a new position.
 *   - {@code holdPosition()}: Returns a Command that holds the claw at the last goal position.
 *   - {@code shiftUp()}: Returns a Command that shifts the claw's position up by a fixed increment.
 *   - {@code shiftDown()}: Returns a Command that shifts the claw's position down by a fixed
 *     increment.
 *   - {@code setGoalPosition(double goal)}: Sets the goal state for the subsystem.
 *   - {@code atGoalPosition()}: Returns whether the claw has reached the goal position.
 *   - {@code enable()}: Enables the PID control of the claw.
 *   - {@code disable()}: Disables the PID control of the claw.
 *   - {@code getMeasurement()}: Returns the claw position for PID control and logging.
 *   - {@code getVoltageCommand()}: Returns the motor commanded voltage.
 *   - {@code loadPreferences()}: Loads the preferences for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 * - Fields:
 *   - {@code private final CANSparkMax motor}: The motor used to control the claw.
 *   - {@code private final RelativeEncoder encoder}: The encoder used to measure the claw's
 *     position.
 *   - {@code private ProfiledPIDController clawController}: The PID controller used to control the
 *     claw's movement.
 *   - {@code private ClawFeedforward feedforward}: The feedforward controller used to calculate the
 *     motor output.
 *   - {@code private double output}: The output of the PID controller.
 *   - {@code private TrapezoidProfile.State setpoint}: The setpoint of the PID controller.
 *   - {@code private double newFeedforward}: The calculated feedforward value.
 *   - {@code private boolean clawEnabled}: A flag indicating whether the claw is enabled.
 *   - {@code private double voltageCommand}: The motor commanded voltage.
 * </pre>
 */
public class ClawSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the claw subsystem. */
  public static class Hardware {
    /** The SparkMax motor used to control the claw. */
    SparkMax motor;

    /** The relative encoder for position feedback. */
    RelativeEncoder encoder;

    /** The absolute encoder for position verification. */
    AbsoluteEncoder absoluteEncoder;

    /**
     * Constructs a Hardware object with the specified motor, encoder, and absolute encoder.
     *
     * @param motor the SparkMax motor
     * @param encoder the relative encoder
     * @param absoluteEncoder the absolute encoder
     */
    public Hardware(SparkMax motor, RelativeEncoder encoder, AbsoluteEncoder absoluteEncoder) {
      this.motor = motor;
      this.encoder = encoder;
      this.absoluteEncoder = absoluteEncoder;
    }
  }

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final AbsoluteEncoder absoluteEncoder;
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  private ProfiledPIDController clawController =
      new ProfiledPIDController(
          ClawConstants.CLAW_KP,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ClawConstants.CLAW_MAX_VELOCITY_RAD_PER_SEC,
              ClawConstants.CLAW_MAX_ACCELERATION_RAD_PER_SEC2));

  private ArmFeedforward feedforward =
      new ArmFeedforward(
          ClawConstants.CLAW_KS,
          ClawConstants.CLAW_KG,
          ClawConstants.CLAW_KV_VOLTS_PER_RAD_PER_SEC,
          0.0); // Acceleration is not used in this implementation

  private double output = 0.0;
  private TrapezoidProfile.State setpoint = new State();
  private double newFeedforward = 0;
  private boolean clawEnabled;
  private double voltageCommand = 0.0;
  private boolean absoluteEncoderValid = true;
  private int absoluteEncoderFreezeCount = 0;
  private double absoluteEncoderLastValue = 0.0;

  // Setup tunable numbers for the claw.
  private TunableNumber kp = new TunableNumber("ClawKP", ClawConstants.CLAW_KP);
  private TunableNumber ks = new TunableNumber("ClawKS", ClawConstants.CLAW_KS);
  private TunableNumber kg = new TunableNumber("ClawKG", ClawConstants.CLAW_KG);
  private TunableNumber kv =
      new TunableNumber("ClawKV", ClawConstants.CLAW_KV_VOLTS_PER_RAD_PER_SEC);
  private TunableNumber maxVelocity =
      new TunableNumber("ClawMaxVelocity", ClawConstants.CLAW_MAX_VELOCITY_RAD_PER_SEC);
  private TunableNumber maxAcceleration =
      new TunableNumber("ClawMaxAcceleration", ClawConstants.CLAW_MAX_ACCELERATION_RAD_PER_SEC2);

  /**
   * Constructs a new ClawSubsystem with the provided hardware components.
   *
   * @param clawHardware the hardware components required for the claw subsystem
   */
  public ClawSubsystem(Hardware clawHardware) {
    this.motor = clawHardware.motor;
    this.encoder = clawHardware.encoder;
    this.absoluteEncoder = clawHardware.absoluteEncoder;

    initializeClaw();
  }

  private void initializeClaw() {
    double clawOffset;

    initMotor();

    clawOffset =
        RobotBase.isReal()
            ? getAbsoluteAngle()
            : Units.radiansToDegrees(ClawConstants.CLAW_OFFSET_RADS);
    DataLogManager.log("Claw Offset Data: " + clawOffset);
    // Set tolerances that will be used to determine when the claw is at the goal position.
    clawController.setTolerance(
        Constants.ClawConstants.POSITION_TOLERANCE, Constants.ClawConstants.VELOCITY_TOLERANCE);

    disable();

    // Add buttons for the dashboard
    SmartDashboard.putData(
        "Claw Brake Mode",
        new InstantCommand(() -> setBrakeMode(true))
            .ignoringDisable(true)
            .withName("Claw Brake Mode"));

    SmartDashboard.putData(
        "Claw Coast Mode",
        new InstantCommand(() -> setBrakeMode(false))
            .ignoringDisable(true)
            .withName("Claw Coast Mode"));

    SmartDashboard.putData(
        "Claw Reset Position",
        new InstantCommand(this::resetEncoder)
            .ignoringDisable(true)
            .withName("Claw Reset Position"));

    SmartDashboard.putData(shiftUp());
    SmartDashboard.putData(shiftDown());

    setDefaultCommand(runOnce(this::disable).andThen(run(() -> {})).withName("Idle"));
  }

  private void initMotor() {
    motorConfig.smartCurrentLimit(ClawConstants.CURRENT_LIMIT);
    motorConfig.inverted(ClawConstants.INVERTED);

    // Setup the encoder scale factors. Since this is a relative encoder,
    // claw position will only be correct if the claw is in the starting rest position when
    // the subsystem is constructed.
    motorConfig
        .encoder
        .positionConversionFactor(ClawConstants.CLAW_RAD_PER_ENCODER_ROTATION)
        .velocityConversionFactor(ClawConstants.RPM_TO_RAD_PER_SEC);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.clearFaults();
    encoder.setPosition(0);

    // Configure the motor to use EMF braking when idle.
    setBrakeMode(true);
    DataLogManager.log("Claw motor firmware version:" + motor.getFirmwareString());
  }

  /**
   * Initialize hardware devices for the claw subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    SparkMax motor = new SparkMax(ClawConstants.MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();
    AbsoluteEncoder absoluteEncoder = motor.getAbsoluteEncoder();

    return new Hardware(motor, encoder, absoluteEncoder);
  }

  @Override
  public void periodic() {

    checkAbsoluteEncoder();

    SmartDashboard.putBoolean("Claw/Enabled", clawEnabled);
    SmartDashboard.putNumber(
        "Claw/Goal", Units.radiansToDegrees(clawController.getGoal().position));
    SmartDashboard.putNumber("Claw/Angle", getRelativeAngle());
    SmartDashboard.putNumber("Claw/Absolute Angle", getAbsoluteAngle());
    SmartDashboard.putNumber("Claw/Combined Angle", Math.toDegrees(getMeasurement()));
    SmartDashboard.putNumber("Claw/Encoder Delta", (getRelativeAngle() - getAbsoluteAngle()));
    SmartDashboard.putNumber("Claw/Voltage", voltageCommand);
    SmartDashboard.putNumber("Claw/Current", motor.getOutputCurrent());
    SmartDashboard.putNumber("Claw/Temp", motor.getMotorTemperature());
    SmartDashboard.putNumber("Claw/SetPt Pos", Units.radiansToDegrees(setpoint.position));
    SmartDashboard.putBoolean("Claw/Encoder Valid", absoluteEncoderValid);
    SmartDashboard.putBoolean("Claw/At Goal", atGoalPosition());

    if (Constants.SD_SHOW_CLAW_EXTENDED_LOGGING_DATA) {
      SmartDashboard.putNumber("Claw/Feedforward", newFeedforward);
      SmartDashboard.putNumber("Claw/PID output", output);
      SmartDashboard.putNumber("Claw/SetPt Vel", Units.radiansToDegrees(setpoint.velocity));
      SmartDashboard.putNumber("Claw/Velocity", Units.radiansToDegrees(encoder.getVelocity()));
    }
  }

  /**
   * Generates the motor command using the PID controller and feedforward.
   *
   * <p>When enabled, calculates the PID output and feedforward to move to the goal position. When
   * disabled, sets all outputs to zero.
   */
  public void useOutput() {
    if (clawEnabled) {
      // Calculate the next set point along the profile to the goal and the next PID output based
      // on the set point and current position.
      output = clawController.calculate(getMeasurement());
      setpoint = clawController.getSetpoint();

      // Calculate the feedforward to move the claw at the desired velocity and offset
      // the effect of gravity at the desired position. Voltage for acceleration is not
      // used.
      newFeedforward = feedforward.calculate(setpoint.position, setpoint.velocity);

      // Add the feedforward to the PID output to get the motor output
      voltageCommand = output + newFeedforward;

    } else {
      // If the claw isn't enabled, set the motor command to 0. In this state the claw
      // will move down until it hits the rest position. Motor EMF braking will slow movement
      // if that mode is used.
      output = 0;
      newFeedforward = 0;
      voltageCommand = 0;
    }
    motor.setVoltage(voltageCommand);
  }

  /**
   * Returns a Command that moves the claw to a new position.
   *
   * @param goal the desired position in radians
   * @return Command that moves the claw to the specified position
   */
  public Command moveToPosition(double goal) {
    return new FunctionalCommand(
        () -> setGoalPosition(goal),
        this::useOutput,
        interrupted -> {},
        this::atGoalPosition,
        this);
  }

  /**
   * Returns a Command that holds the claw at the last goal position using the PID Controller
   * driving the motor.
   *
   * @return Command that holds the claw at its current goal position
   */
  public Command holdPosition() {
    return run(this::useOutput).withName("Claw: Hold Position");
  }

  /**
   * Creates an abort command that sets the claw position to the current position in any restricted
   * areas.
   *
   * @return InstantCommand that stops movement and sets goal to current position
   */
  public Command abortCommand() {
    return new InstantCommand(
        () -> {
          setGoalPosition(getMeasurement());
          clawController.reset(getMeasurement());
        });
  }

  /**
   * Returns a Command that shifts claw position up by a fixed increment.
   *
   * @return Command that shifts the claw position upward by POS_INCREMENT
   */
  public Command shiftUp() {
    return runOnce(
            () ->
                setGoalPosition(
                    clawController.getGoal().position + Constants.ClawConstants.POS_INCREMENT))
        .andThen(run(this::useOutput))
        .until(this::atGoalPosition)
        .withName("Claw: Shift Position Up");
  }

  /**
   * Returns a Command that shifts claw position down by a fixed increment.
   *
   * @return Command that shifts the claw position downward by POS_INCREMENT
   */
  public Command shiftDown() {
    return runOnce(
            () ->
                setGoalPosition(
                    clawController.getGoal().position - Constants.ClawConstants.POS_INCREMENT))
        .andThen(run(this::useOutput))
        .until(this::atGoalPosition)
        .withName("Claw: Shift Position Down");
  }

  /**
   * Set the goal state for the subsystem, limited to allowable range. Goal velocity is set to zero.
   * The ProfiledPIDController drives the claw to this position and holds it there.
   *
   * @param goal the desired goal position in radians
   */
  private void setGoalPosition(double goal) {
    clawController.setGoal(
        new TrapezoidProfile.State(
            MathUtil.clamp(
                goal,
                Constants.ClawConstants.MIN_ANGLE_RADS,
                Constants.ClawConstants.MAX_ANGLE_RADS),
            0));

    // Call enable() to configure and start the controller in case it is not already enabled.
    enable();
  }

  /**
   * Returns whether the claw has reached the goal position and velocity is within limits.
   *
   * @return true if the claw is at the goal position and velocity within tolerance, false otherwise
   */
  public boolean atGoalPosition() {
    return clawController.atGoal();
  }

  /**
   * Sets up the PID controller to move the claw to the defined goal position and hold at that
   * position. Preferences for tuning the controller are applied.
   */
  private void enable() {

    // Don't enable if already enabled since this may cause control transients
    if (!clawEnabled) {
      loadTunableNumbers();
      setDefaultCommand(holdPosition());

      // Reset the PID controller to clear any previous state
      clawController.reset(getMeasurement());
      clawEnabled = true;

      DataLogManager.log(
          "Claw Enabled - kP="
              + clawController.getP()
              + " kI="
              + clawController.getI()
              + " kD="
              + clawController.getD()
              + " PosGoal="
              + Units.radiansToDegrees(clawController.getGoal().position)
              + " CurPos="
              + Units.radiansToDegrees(getMeasurement()));
    }
  }

  /**
   * Disables the PID control of the claw. Sets motor output to zero. NOTE: In this state the claw
   * will move until it hits the stop. Using EMF braking mode with motor will slow this movement.
   */
  public void disable() {

    // Clear the enabled flag and call useOutput to zero the motor command
    clawEnabled = false;
    useOutput();
    setDefaultCommand(run(() -> {}).withName("Idle"));

    DataLogManager.log(
        "Claw Disabled CurPos="
            + Units.radiansToDegrees(getMeasurement())
            + " CurVel="
            + Units.radiansToDegrees(encoder.getVelocity()));
  }

  /**
   * Returns the relative claw angle using the built-in encoder.
   *
   * @return the current claw angle in degrees relative to the encoder zero position
   */
  public double getRelativeAngle() {
    return Math.toDegrees(encoder.getPosition() + ClawConstants.CLAW_OFFSET_RADS);
  }

  /**
   * Returns the claw position for PID control and logging.
   *
   * @return the claw position in radians from horizontal (corrected using absolute or relative
   *     encoder)
   */
  public double getMeasurement() {
    if (absoluteEncoderValid) {
      return Math.toRadians(getAbsoluteAngle());
    } else {
      // If the absolute encoder is not valid, use the relative encoder.
      return encoder.getPosition() + ClawConstants.CLAW_OFFSET_RADS;
    }
  }

  /**
   * Returns the absolute claw angle.
   *
   * @return the current claw angle in degrees from the absolute encoder
   */
  public double getAbsoluteAngle() {
    // Add the offset from the 0 point of the encoder.
    double angle = absoluteEncoder.getPosition() * 360 - ClawConstants.ABSOLUTE_OFFSET_DEGREES;
    if (angle < 0) {
      angle += 360;
    }
    return angle;
  }

  /**
   * Checks the absolute encoder for freeze condition. Due to noise, a normally functioning encoder
   * will change value continually. This method detects when the encoder appears frozen.
   */
  private void checkAbsoluteEncoder() {

    if (RobotBase.isReal()) {

      Double absoluteEncoderValue = absoluteEncoder.getPosition();
      if (absoluteEncoderValue.equals(absoluteEncoderLastValue)) {
        absoluteEncoderFreezeCount++;
        if (absoluteEncoderFreezeCount > 20) {
          absoluteEncoderValid = false;
        }
      } else {
        absoluteEncoderFreezeCount = 0;
      }
      absoluteEncoderLastValue = absoluteEncoderValue;
    }
  }

  /**
   * Returns the motor commanded voltage.
   *
   * @return the voltage command currently being sent to the motor in volts
   */
  public double getVoltageCommand() {
    return voltageCommand;
  }

  /**
   * Returns the motor for simulation.
   *
   * @return the SparkMax motor used for simulation purposes
   */
  public SparkMax getMotor() {
    return motor;
  }

  /**
   * Set the motor idle mode to brake or coast.
   *
   * @param enableBrake Enable motor braking when idle
   */
  public void setBrakeMode(boolean enableBrake) {
    SparkMaxConfig brakeConfig = new SparkMaxConfig();
    if (enableBrake) {
      DataLogManager.log("Claw motor set to brake mode");
      brakeConfig.idleMode(IdleMode.kBrake);
    } else {
      DataLogManager.log("Claw motor set to coast mode");
      brakeConfig.idleMode(IdleMode.kCoast);
    }
    motor.configure(
        brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Reset the encoder to the start (zero) position. This should only be done when the claw is
   * resting against the back stop. This command doesn't work in simulation.
   */
  public void resetEncoder() {
    DataLogManager.log("Claw encoder reset");
    encoder.setPosition(0);
  }

  /**
   * Checks if the claw is enabled or not.
   *
   * @return true if PID control is enabled for the claw, false otherwise
   */
  public boolean isEnabled() {
    return clawEnabled;
  }

  /**
   * Load values that can be tuned at runtime. This should only be called when the controller is
   * disabled - for example from enable().
   */
  private void loadTunableNumbers() {

    // Read Preferences for PID controller
    clawController.setP(kp.get());

    // Read Preferences for Trapezoid Profile and update
    clawController.setConstraints(
        new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

    // Read Preferences for Feedforward and create a new instance
    feedforward = new ArmFeedforward(ks.get(), kg.get(), kv.get(), 0);
  }

  /** Closes any Closable objects including the motor. */
  @Override
  public void close() {
    motor.close();
  }
}
