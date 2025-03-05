// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RollerConstants;
import frc.robot.util.TunableNumber;

/**
 * The {@code RollerSubsystem} class is a subsystem that controls the speed of an Roller using a PID
 * Controller and simple motor feedforward. It uses a CANSparkMax motor and a RelativeEncoder to
 * measure the Roller's speed. The class provides methods to return commands that run the Roller at
 * the specified speed or stop the motor.
 *
 * <p>The RollerSubsystem class provides a constructor where hardware dependencies are passed in to
 * allow access for testing. There is also a method provided to create default hardware when those
 * details are not needed outside of the subsystem.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of RollerSubsystem using specified hardware
 * SparkMax rollerMotor = new SparkMax(1, MotorType.kBrushless);
 * RelativeEncoder rollerEncoder = rollerMotor.getEncoder();
 * CANrange canRange = new CANrange(2);
 * RollerHardware = new RollerSubsystem.Hardware(motor, encoder, canRange;
 * RollerSubsystem RollerSubsystem = new RollerSubsystem(RollerHardware);
 *
 * // Create a new instance of RollerSubsystem using default hardware
 * RollerSubsystem RollerSubsystem = new RollerSubsystem(initializeHardware());
 *
 * // Run the Roller forward at the defined speed
 * Command runRollerCommand = RollerSubsystem.runForward();
 * runRollerCommand.schedule();
 *
 * }
 *
 * Code Analysis:
 * - Main functionalities:
 *   - Control the speed of an Roller using a PID Controller
 * - Methods:
 *   - {@code periodic()}: Publish telemetry with information about the Roller's state.
 *   - {@code updateMotorController()}: Generates the motor command using the PID controller and
 *     feedforward.
 *   - {@code runForward()}: Returns a Command that runs the motor forward at the current set
 *     speed.
 *   - {@code runReverse()}: Returns a Command that runs the motor in reverse at the current set
 *     speed.
 *   - {@code setMotorSetPoint(double scale)}: Set the setpoint for the motor as a scale factor
 *     applied to the setpoint value.
 *   - {@code RollerAtSetpoint()}: Returns whether the Roller has reached the set point velocity
 *     within limits.
 *   - {@code enableRoller()}: Enables the PID control of the Roller.
 *   - {@code disableRoller()}: Disables the PID control of the Roller.
 *   - {@code getRollerSpeed()}: Returns the Roller position for PID control and logging.
 *   - {@code getRollerVoltageCommand()}: Returns the motor commanded voltage.
 *   - {@code loadTunableNumbers()}: Loads the tunable numbers for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 * </pre>
 */
public class RollerSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the Roller subsystem. */
  public static class Hardware {
    SparkMax motor;
    RelativeEncoder encoder;
    CANrange canRange;

    /** Create the Hardware object using the specified hardware objects. */
    public Hardware(SparkMax motor, RelativeEncoder encoder, CANrange canRange) {
      this.motor = motor;
      this.encoder = encoder;
      this.canRange = canRange;
    }
  }

  private final CANrange canRange = new CANrange(RollerConstants.CANRANGE_PORT);

  private final SparkMax rollerMotor;
  private final RelativeEncoder rollerEncoder;
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  private PIDController speedController = new PIDController(RollerConstants.SPEED_KP, 0.0, 0.0);
  private PIDController positionController =
      new PIDController(RollerConstants.POSITION_KP, 0.0, 0.0);

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          RollerConstants.SPEED_KS_VOLTS, RollerConstants.SPEED_KV_VOLTS_PER_RPM, 0.0);

  private double pidOutput = 0.0;
  private double newFeedforward = 0;
  private boolean rollerEnabled;
  private double rollerVoltageCommand = 0.0;

  // Setup tunable numbers for the Roller.
  private TunableNumber positionKp =
      new TunableNumber("RollerPositionKP", RollerConstants.POSITION_KP);
  private TunableNumber speedKp = new TunableNumber("RollerSpeedKP", RollerConstants.SPEED_KP);
  private TunableNumber speedKs =
      new TunableNumber("RollerSpeedKS", RollerConstants.SPEED_KS_VOLTS);
  private TunableNumber speedKv =
      new TunableNumber("RollerSpeedKV", RollerConstants.SPEED_KV_VOLTS_PER_RPM);
  private TunableNumber forwardSetSpeed =
      new TunableNumber("RollerForwardRPM", RollerConstants.ROLLER_SET_POINT_FORWARD_RPM);
  private TunableNumber reverseSetSpeed =
      new TunableNumber("RollerReverseRPM", RollerConstants.ROLLER_SET_POINT_REVERSE_RPM);
  private TunableNumber loadSetSpeed =
      new TunableNumber("RollerLoadRPM", RollerConstants.ROLLER_LOAD_CORAL_RPM);

  // Flag to reset the Roller position in simulation
  private boolean resetSimPosition = false;

  /** Create a new RollerSubsystem controlled by a Profiled PID COntroller . */
  public RollerSubsystem(Hardware rollerHardware) {
    this.rollerMotor = rollerHardware.motor;
    this.rollerEncoder = rollerHardware.encoder;

    initializeRoller();
  }

  private void initializeRoller() {

    initRollerMotor();
    initializeCANRange();

    // Set tolerances that will be used to determine when the Roller is at the goal velocity.
    speedController.setTolerance(RollerConstants.ROLLER_TOLERANCE_RPM);

    disableRoller();

    setDefaultCommand(holdPosition().withName("Hold Position"));
  }

  private void initRollerMotor() {

    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(RollerConstants.CURRENT_LIMIT);

    // Setup the encoder scale factors
    motorConfig.encoder.velocityConversionFactor(
        RollerConstants.ROLLER_ROTATIONS_PER_ENCODER_ROTATION);
    motorConfig.encoder.positionConversionFactor(
        RollerConstants.ROLLER_ROTATIONS_PER_ENCODER_ROTATION);

    rollerMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerMotor.clearFaults();

    DataLogManager.log("Roller motor firmware version:" + rollerMotor.getFirmwareString());
  }

  /**
   * Create hardware devices for the Roller subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    SparkMax rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder rollerEncoder = rollerMotor.getEncoder();
    CANrange canRange = new CANrange(RollerConstants.CANRANGE_PORT);

    return new Hardware(rollerMotor, rollerEncoder, canRange);
  }

  // Initialize CANRange sensor that detects coral in the Roller
  private void initializeCANRange() {

    // Add a dashboard value for testing purposes to simulate a coral being loaded
    SmartDashboard.putBoolean("Force Coral Loaded", false);

    // Set the CANRange sensor configuration to avoid false detects from nearby objects
    var toApply = new CANrangeConfiguration();
    toApply.ProximityParams.MinSignalStrengthForValidMeasurement = 10000;
    toApply.ProximityParams.ProximityThreshold = 0.1;
    canRange.getConfigurator().apply(toApply);

    /* Set the signal update rate */
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, canRange.getDistance(), canRange.getSignalStrength(), canRange.getIsDetected());
  }

  /**
   * Check if coral is detected by the CANRange sensor.
   *
   * @return Coral is detected
   */
  public boolean isCoralInsideRoller() {
    // For simulation purposes use a dashboard value to trip the sensor
    return RobotBase.isReal()
        ? canRange.getIsDetected().getValue()
        : SmartDashboard.getBoolean("Force Coral Loaded", false);
  }

  /** Publish telemetry with information about the Roller's state. */
  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Roller/Enabled", rollerEnabled);
    SmartDashboard.putNumber("Roller/Setpoint", speedController.getSetpoint());
    SmartDashboard.putNumber("Roller/Speed", getRollerSpeed());
    SmartDashboard.putNumber("Roller/Rotations", getRollerPosition());
    SmartDashboard.putNumber("Roller/Voltage", rollerVoltageCommand);
    SmartDashboard.putNumber("Roller/Temperature", rollerMotor.getMotorTemperature());
    SmartDashboard.putNumber("Roller/Current", rollerMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Roller/Detected", isCoralInsideRoller());

    if (Constants.SD_SHOW_ROLLER_EXTENDED_LOGGING_DATA) {
      SmartDashboard.putNumber("Roller/Feedforward", newFeedforward);
      SmartDashboard.putNumber("Roller/PID output", pidOutput);
      SmartDashboard.putNumber(
          "Roller/Detector Distance", canRange.getDistance().getValueAsDouble());
      SmartDashboard.putNumber(
          "Roller/Detector Strength", canRange.getSignalStrength().getValueAsDouble());
    }
  }

  /** Generate the motor command using the speed PID controller output and feedforward. */
  public void updateSpeedController() {
    if (rollerEnabled) {
      // Calculate the the motor command by adding the PID controller output and feedforward to run
      // the Roller at the desired speed. Store the individual values for logging.
      pidOutput = speedController.calculate(getRollerSpeed());
      newFeedforward = feedforward.calculate(speedController.getSetpoint());
      rollerVoltageCommand = pidOutput + newFeedforward;

    } else {
      // If the Roller isn't enabled, set the motor command to 0. In this state the Roller
      // will slow down until it stops. Motor EMF braking will cause it to slow down faster
      // if that mode is used.
      pidOutput = 0;
      newFeedforward = 0;
      rollerVoltageCommand = 0;
    }
    rollerMotor.setVoltage(rollerVoltageCommand);
  }

  /** Generate the motor command using the position PID controller output. */
  public void updatePositionController() {
    if (rollerEnabled) {
      // Calculate the the motor command using the PID controller output to hold
      // the Roller at the desired position. Store the individual values for logging.
      pidOutput = positionController.calculate(getRollerPosition());
      rollerVoltageCommand = pidOutput;

    } else {
      // If the Roller isn't enabled, set the motor command to 0. In this state the Roller
      // will slow down until it stops. Motor EMF braking will cause it to slow down faster
      // if that mode is used.
      pidOutput = 0;
      rollerVoltageCommand = 0;
    }
    rollerMotor.setVoltage(rollerVoltageCommand);
  }

  /** Returns a Command that runs the motor forward at the current set speed. */
  public Command runForward() {
    return new FunctionalCommand(
        () -> startSpeedControl(forwardSetSpeed),
        this::updateSpeedController,
        interrupted -> disableRoller(),
        () -> false,
        this);
  }

  /**
   * Loads Coral until CANRange detects Coral, runs for a short time after and then holds position.
   */
  public Command loadCoral() {
    return Commands.sequence(
        runLoadCoral().until(this::isCoralInsideRoller), runLoadCoral().withTimeout(0.1));
  }

  /** Returns a Command that runs the motor in reverse to load coral. */
  public Command runLoadCoral() {
    return new FunctionalCommand(
        () -> startSpeedControl(loadSetSpeed),
        this::updateSpeedController,
        interrupted -> disableRoller(),
        () -> false,
        this);
  }

  /** Returns a Command that runs the motor in reverse at the current set speed. */
  public Command runReverse() {
    return new FunctionalCommand(
        () -> startSpeedControl(reverseSetSpeed),
        this::updateSpeedController,
        interrupted -> disableRoller(),
        () -> false,
        this);
  }

  /** Returns a Command that holds the motor at the current position. */
  public Command holdPosition() {
    return new FunctionalCommand(
        this::startPositionControl,
        this::updatePositionController,
        interrupted -> disableRoller(),
        () -> false,
        this);
  }

  /**
   * Set the setpoint for motor speed and start the motor. The PIDController drives the motor to
   * this speed and holds it there.
   */
  private void startSpeedControl(TunableNumber speed) {
    loadTunableNumbers();
    speedController.setSetpoint(speed.get());

    // Call enable() to configure and start the controller in case it is not already enabled.
    enableRoller();
  }

  /**
   * Set the setpoint for the motor and start the motor. The PIDController drives the motor to zero
   * position and holds it there.
   */
  private void startPositionControl() {
    loadTunableNumbers();
    rollerEncoder.setPosition(0);
    positionController.setSetpoint(0.0);

    // Set a flag to tell simulation to reset the position
    resetSimPosition = true;

    // Call enable() to configure and start the controller in case it is not already enabled.
    enableRoller();
  }

  /** Returns whether the Roller has reached the set point speed within limits. */
  public boolean rollerAtSetpoint() {
    return speedController.atSetpoint();
  }

  /**
   * Sets up the PID controller to run the Roller at the defined setpoint speed. Preferences for
   * tuning the controller are applied.
   */
  private void enableRoller() {

    // Don't enable if already enabled since this may cause control transients
    if (!rollerEnabled) {
      loadTunableNumbers();

      // Reset the PID controller to clear any previous state
      speedController.reset();
      positionController.reset();
      rollerEnabled = true;

      DataLogManager.log(
          "Roller Enabled - Speed kP="
              + speedController.getP()
              + " Position kP="
              + positionController.getP()
              + " Setpoint="
              + speedController.getSetpoint()
              + " CurSpeed="
              + getRollerSpeed());
    }
  }

  /**
   * Disables the PID control of the Roller. Sets motor output to zero. NOTE: In this state the
   * Roller will slow down until it stops. Motor EMF braking will cause it to slow down faster if
   * that mode is used.
   */
  public void disableRoller() {

    // Clear the enabled flag and update the controller to zero the motor command
    rollerEnabled = false;
    updateSpeedController();

    DataLogManager.log("Roller Disabled CurSpeed=" + getRollerSpeed());
  }

  /** Returns the Roller speed for PID control and logging (Units are RPM). */
  public double getRollerSpeed() {
    return rollerEncoder.getVelocity();
  }

  /** Returns the Roller position (Units are number of wheel rotations). */
  public double getRollerPosition() {
    return rollerEncoder.getPosition();
  }

  /** Returns the Roller motor commanded voltage. */
  public double getRollerVoltageCommand() {
    return rollerVoltageCommand;
  }

  /** Returns the motor for simulation. */
  public SparkMax getMotor() {
    return rollerMotor;
  }

  /**
   * Load values that can be tuned at runtime. This should only be called when the controller is
   * disabled - for example from enable().
   */
  private void loadTunableNumbers() {

    // Read values for PID controllerS
    speedController.setP(speedKp.get());
    positionController.setP(positionKp.get());

    // Read values for Feedforward and create a new instance
    feedforward = new SimpleMotorFeedforward(speedKs.get(), speedKv.get(), 0.0);
  }

  /** Get the flag to reset the Roller position in simulation. */
  public boolean getResetSimPosition() {
    return resetSimPosition;
  }

  /** Clear the flag to reset the Roller position in simulation. */
  public void clearResetSimPosition() {
    resetSimPosition = false;
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    rollerMotor.close();
  }
}
