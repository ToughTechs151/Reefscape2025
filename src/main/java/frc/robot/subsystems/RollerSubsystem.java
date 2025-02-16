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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
 * CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
 * RelativeEncoder encoder = motor.getEncoder();
 * RollerHardware = new RollerSubsystem.Hardware(motor, encoder);
 * RollerSubsystem RollerSubsystem = new RollerSubsystem(RollerHardware);
 *
 * // Create a new instance of RollerSubsystem using default hardware
 * RollerSubsystem RollerSubsystem = new RollerSubsystem(initializeHardware());
 *
 * // Run the Roller at a specific speed
 * Command runRollerCommand = RollerSubsystem.runRoller(500.0);
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
 *   - {@code loadPreferences()}: Loads the preferences for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 *   - Fields:
 *   - {@code private final CANSparkMax motor}: The motor used to control the Roller.
 *   - {@code private final RelativeEncoder encoder}: The encoder used to measure the Roller's
 *     position.
 *   - {@code private PIDController RollerController}: The PID controller used to
 *     control the Roller's speed.
 *   - {@code private Feedforward feedforward}: The feedforward controller used to
 *     calculate the motor output.
 *   - {@code private double pidOutput}: The output of the PID controller.
 *   - {@code private double newFeedforward}: The calculated feedforward value.
 *   - {@code private boolean RollerEnabled}: A flag indicating whether the Roller is enabled.
 *   - {@code private double RollerVoltageCommand}: The motor commanded voltage.
 *   - {@code private double setSpeed}: The setpoint speed for the controller.
 * </pre>
 */
public class RollerSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the Roller subsystem. */
  public static class Hardware {
    SparkMax motor;
    RelativeEncoder encoder;

    public Hardware(SparkMax motor, RelativeEncoder encoder) {
      this.motor = motor;
      this.encoder = encoder;
    }
  }

  private final CANrange canRange = new CANrange(RollerConstants.CANRANGE_PORT);
  private DigitalInput canRangeDigitalInput;

  private final SparkMax rollerMotor;
  private final RelativeEncoder rollerEncoder;
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  private PIDController rollerController = new PIDController(RollerConstants.ROLLER_KP, 0.0, 0.0);

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          RollerConstants.ROLLER_KS_VOLTS, RollerConstants.ROLLER_KV_VOLTS_PER_RPM, 0.0);

  private double pidOutput = 0.0;
  private double newFeedforward = 0;
  private boolean rollerEnabled;
  private double rollerVoltageCommand = 0.0;

  // Setup tunable numbers for the Roller.
  private TunableNumber kp = new TunableNumber("RollerKP", RollerConstants.ROLLER_KP);
  private TunableNumber ks = new TunableNumber("RollerKS", RollerConstants.ROLLER_KS_VOLTS);
  private TunableNumber kv = new TunableNumber("RollerKV", RollerConstants.ROLLER_KV_VOLTS_PER_RPM);
  private TunableNumber forwardSetSpeed =
      new TunableNumber("RollerForwardRPM", RollerConstants.ROLLER_SET_POINT_FORWARD_RPM);
  private TunableNumber reverseSetSpeed =
      new TunableNumber("RollerReverseRPM", RollerConstants.ROLLER_SET_POINT_REVERSE_RPM);
  private TunableNumber speedThreshold =
      new TunableNumber("RollerThresholdRPM", RollerConstants.ROLLER_SPEED_THRESHOLD_RPM);
  private TunableNumber currentThreshold =
      new TunableNumber("RollerThresholdAmps", RollerConstants.ROLLER_CURRENT_THRESHOLD_AMPS);

  /** Create a new RollerSubsystem controlled by a Profiled PID COntroller . */
  public RollerSubsystem(Hardware rollerHardware) {
    this.rollerMotor = rollerHardware.motor;
    this.rollerEncoder = rollerHardware.encoder;

    initializeRoller();
  }

  private void initializeRoller() {

    initRollerMotor();

    // Set tolerances that will be used to determine when the Roller is at the goal velocity.
    rollerController.setTolerance(RollerConstants.ROLLER_TOLERANCE_RPM);

    disableRoller();

    setDefaultCommand(runOnce(this::disableRoller).andThen(run(() -> {})).withName("Idle"));
  }

  private void initRollerMotor() {

    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(RollerConstants.CURRENT_LIMIT);

    // Setup the encoder scale factors
    motorConfig.encoder.velocityConversionFactor(
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

    return new Hardware(rollerMotor, rollerEncoder);
  }

  public void initializeCan() {

    // Initialize Beam Breaker
    canRangeDigitalInput = new DigitalInput(RollerConstants.CANRANGE_PORT);
    SmartDashboard.putBoolean("Force Coral Loaded", false);

    /* Configure CANcoder */
    var toApply = new CANrangeConfiguration();

    /* User can change the configs if they want, or leave it empty for factory-default */
    canRange.getConfigurator().apply(toApply);

    /* Set the signal update rate */
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, canRange.getDistance(), canRange.getSignalStrength(), canRange.getIsDetected());
  }

  public boolean isCoralInsideRoller() {
    // For test purposes also allow a dashboard value to tri[p] the sensor
    return !canRangeDigitalInput.get() || SmartDashboard.getBoolean("Force Coral Loaded", false);
  }

  /** Publish telemetry with information about the Roller's state. */
  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Roller Enabled", rollerEnabled);
    SmartDashboard.putNumber("Roller Setpoint", rollerController.getSetpoint());
    SmartDashboard.putNumber("Roller Speed", rollerEncoder.getVelocity());
    SmartDashboard.putNumber("Roller Voltage", rollerVoltageCommand);
    SmartDashboard.putNumber("Roller Temp", rollerMotor.getMotorTemperature());
    SmartDashboard.putNumber("Roller Current", rollerMotor.getOutputCurrent());

    if (Constants.SD_SHOW_ROLLER_EXTENDED_LOGGING_DATA) {
      SmartDashboard.putNumber("Roller Feedforward", newFeedforward);
      SmartDashboard.putNumber("Roller PID output", pidOutput);
    }

    // Get distance, signal strength and detected. Get calls automatically call refresh()
    // , no need to manually refresh.

    var distance = canRange.getDistance();
    SmartDashboard.putNumber("Distance", distance.getValueAsDouble());

    var strength = canRange.getSignalStrength();
    SmartDashboard.putNumber("Strength", strength.getValueAsDouble());

    boolean detected = canRange.getIsDetected().getValue();
    SmartDashboard.putBoolean("Detected", detected);
  }

  /** Generate the motor command using the PID controller output and feedforward. */
  public void updateMotorController() {
    if (rollerEnabled) {
      // Calculate the the motor command by adding the PID controller output and feedforward to run
      // the Roller at the desired speed. Store the individual values for logging.
      pidOutput = rollerController.calculate(getRollerSpeed());
      newFeedforward = feedforward.calculate(rollerController.getSetpoint());
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

  /** Returns a Command that runs the motor forward at the current set speed. */
  public Command runForward() {
    return new FunctionalCommand(
        this::setMotorSetPointForward,
        this::updateMotorController,
        interrupted -> disableRoller(),
        () -> false,
        this);
  }

  /** Returns a Command that runs the motor forward until a note is loaded. */
  public Command loadNote() {
    return new FunctionalCommand(
        this::setMotorSetPointForward,
        this::updateMotorController,
        interrupted -> disableRoller(),
        this::noteFullyLoaded,
        this);
  }

  /** Returns a Command that runs the motor in reverse at the current set speed. */
  public Command runReverse() {
    return new FunctionalCommand(
        this::setMotorSetPointReverse,
        this::updateMotorController,
        interrupted -> disableRoller(),
        () -> false,
        this);
  }

  /**
   * Set the setpoint for the motor as a scale factor applied to the setpoint value going forwards.
   * The PIDController drives the motor to this speed and holds it there.
   */
  private void setMotorSetPointForward() {
    loadTunableNumbers();
    rollerController.setSetpoint(forwardSetSpeed.get());

    // Call enable() to configure and start the controller in case it is not already enabled.
    enableRoller();
  }

  /**
   * Set the setpoint for the motor as a scale factor applied to the setpoint value reversed. The
   * PIDController drives the motor to this speed and holds it there.
   */
  private void setMotorSetPointReverse() {
    loadTunableNumbers();
    rollerController.setSetpoint(reverseSetSpeed.get());

    // Call enable() to configure and start the controller in case it is not already enabled.
    enableRoller();
  }

  /** Returns whether the Roller has reached the set point speed within limits. */
  public boolean rollerAtSetpoint() {
    return rollerController.atSetpoint();
  }

  /**
   * Returns whether the Roller has fully pulled in a note which is detected by the motor being up
   * to speed and the current hitting a threshold.
   */
  public boolean noteFullyLoaded() {
    return ((Math.abs(rollerEncoder.getVelocity()) >= speedThreshold.get())
        && (rollerMotor.getOutputCurrent() > currentThreshold.get()));
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
      rollerController.reset();
      rollerEnabled = true;

      DataLogManager.log(
          "Roller Enabled - kP="
              + rollerController.getP()
              + " kI="
              + rollerController.getI()
              + " kD="
              + rollerController.getD()
              + " Setpoint="
              + rollerController.getSetpoint()
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
    updateMotorController();

    DataLogManager.log("Roller Disabled CurSpeed=" + getRollerSpeed());
  }

  /** Returns the Roller speed for PID control and logging (Units are RPM). */
  public double getRollerSpeed() {
    return rollerEncoder.getVelocity();
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

    // Read values for PID controller
    rollerController.setP(kp.get());

    // Read values for Feedforward and create a new instance
    feedforward = new SimpleMotorFeedforward(ks.get(), kv.get(), 0.0);
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    rollerMotor.close();
  }
}
