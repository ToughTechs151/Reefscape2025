// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.TunableNumber;

/**
 * The {@code IntakeSubsystem} class is a subsystem that controls the speed of an intake using a PID
 * Controller and simple motor feedforward. It uses a CANSparkMax motor and a RelativeEncoder to
 * measure the intake's speed. The class provides methods to return commands that run the intake at
 * the specified speed or stop the motor.
 *
 * <p>The IntakeSubsystem class provides a constructor where hardware dependencies are passed in to
 * allow access for testing. There is also a method provided to create default hardware when those
 * details are not needed outside of the subsystem.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of IntakeSubsystem using specified hardware
 * CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
 * RelativeEncoder encoder = motor.getEncoder();
 * intakeHardware = new IntakeSubsystem.Hardware(motor, encoder);
 * IntakeSubsystem intakeSubsystem = new IntakeSubsystem(intakeHardware);
 *
 * // Create a new instance of IntakeSubsystem using default hardware
 * IntakeSubsystem intakeSubsystem = new IntakeSubsystem(initializeHardware());
 *
 * // Run the intake at a specific speed
 * Command runIntakeCommand = intakeSubsystem.runIntake(500.0);
 * runIntakeCommand.schedule();
 *
 * }
 *
 * Code Analysis:
 * - Main functionalities:
 *   - Control the speed of an intake using a PID Controller
 * - Methods:
 *   - {@code periodic()}: Publish telemetry with information about the intake's state.
 *   - {@code updateMotorController()}: Generates the motor command using the PID controller and
 *     feedforward.
 *   - {@code runForward()}: Returns a Command that runs the motor forward at the current set
 *     speed.
 *   - {@code runReverse()}: Returns a Command that runs the motor in reverse at the current set
 *     speed.
 *   - {@code setMotorSetPoint(double scale)}: Set the setpoint for the motor as a scale factor
 *     applied to the setpoint value.
 *   - {@code intakeAtSetpoint()}: Returns whether the intake has reached the set point velocity
 *     within limits.
 *   - {@code enableIntake()}: Enables the PID control of the intake.
 *   - {@code disableIntake()}: Disables the PID control of the intake.
 *   - {@code getIntakeSpeed()}: Returns the intake position for PID control and logging.
 *   - {@code getIntakeVoltageCommand()}: Returns the motor commanded voltage.
 *   - {@code loadPreferences()}: Loads the preferences for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 *   - Fields:
 *   - {@code private final CANSparkMax motor}: The motor used to control the intake.
 *   - {@code private final RelativeEncoder encoder}: The encoder used to measure the intake's
 *     position.
 *   - {@code private PIDController intakeController}: The PID controller used to
 *     control the intake's speed.
 *   - {@code private Feedforward feedforward}: The feedforward controller used to
 *     calculate the motor output.
 *   - {@code private double pidOutput}: The output of the PID controller.
 *   - {@code private double newFeedforward}: The calculated feedforward value.
 *   - {@code private boolean intakeEnabled}: A flag indicating whether the intake is enabled.
 *   - {@code private double intakeVoltageCommand}: The motor commanded voltage.
 *   - {@code private double setSpeed}: The setpoint speed for the controller.
 * </pre>
 */
public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the intake subsystem. */
  public static class Hardware {
    SparkMax motor;
    RelativeEncoder encoder;

    public Hardware(SparkMax motor, RelativeEncoder encoder) {
      this.motor = motor;
      this.encoder = encoder;
    }
  }

  private final SparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  private PIDController intakeController = new PIDController(IntakeConstants.INTAKE_KP, 0.0, 0.0);

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          IntakeConstants.INTAKE_KS_VOLTS, IntakeConstants.INTAKE_KV_VOLTS_PER_RPM, 0.0);

  private double pidOutput = 0.0;
  private double newFeedforward = 0;
  private boolean intakeEnabled;
  private double intakeVoltageCommand = 0.0;

  // Setup tunable numbers for the intake.
  private TunableNumber kp = new TunableNumber("IntakeKP", IntakeConstants.INTAKE_KP);
  private TunableNumber ks = new TunableNumber("IntakeKS", IntakeConstants.INTAKE_KS_VOLTS);
  private TunableNumber kv = new TunableNumber("IntakeKV", IntakeConstants.INTAKE_KV_VOLTS_PER_RPM);
  private TunableNumber forwardSetSpeed =
      new TunableNumber("IntakeForwardRPM", IntakeConstants.INTAKE_SET_POINT_FORWARD_RPM);
  private TunableNumber reverseSetSpeed =
      new TunableNumber("IntakeReverseRPM", IntakeConstants.INTAKE_SET_POINT_REVERSE_RPM);
  private TunableNumber speedThreshold =
      new TunableNumber("IntakeThresholdRPM", IntakeConstants.INTAKE_SPEED_THRESHOLD_RPM);
  private TunableNumber currentThreshold =
      new TunableNumber("IntakeThresholdAmps", IntakeConstants.INTAKE_CURRENT_THRESHOLD_AMPS);

  /** Create a new IntakeSubsystem controlled by a Profiled PID COntroller . */
  public IntakeSubsystem(Hardware intakeHardware) {
    this.intakeMotor = intakeHardware.motor;
    this.intakeEncoder = intakeHardware.encoder;

    initializeIntake();
  }

  private void initializeIntake() {

    initIntakeMotor();

    // Set tolerances that will be used to determine when the intake is at the goal velocity.
    intakeController.setTolerance(IntakeConstants.INTAKE_TOLERANCE_RPM);

    disableIntake();

    setDefaultCommand(runOnce(this::disableIntake).andThen(run(() -> {})).withName("Idle"));
  }

  private void initIntakeMotor() {

    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT);

    // Setup the encoder scale factors
    motorConfig.encoder.velocityConversionFactor(
        IntakeConstants.INTAKE_ROTATIONS_PER_ENCODER_ROTATION);

    intakeMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeMotor.clearFaults();

    DataLogManager.log("Intake motor firmware version:" + intakeMotor.getFirmwareString());
  }

  /**
   * Create hardware devices for the intake subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    SparkMax intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    return new Hardware(intakeMotor, intakeEncoder);
  }

  /** Publish telemetry with information about the intake's state. */
  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Intake Enabled", intakeEnabled);
    SmartDashboard.putNumber("Intake Setpoint", intakeController.getSetpoint());
    SmartDashboard.putNumber("Intake Speed", intakeEncoder.getVelocity());
    SmartDashboard.putNumber("Intake Voltage", intakeVoltageCommand);
    SmartDashboard.putNumber("Intake Temp", intakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());

    if (Constants.SD_SHOW_INTAKE_EXTENDED_LOGGING_DATA) {
      SmartDashboard.putNumber("Intake Feedforward", newFeedforward);
      SmartDashboard.putNumber("Intake PID output", pidOutput);
    }
  }

  /** Generate the motor command using the PID controller output and feedforward. */
  public void updateMotorController() {
    if (intakeEnabled) {
      // Calculate the the motor command by adding the PID controller output and feedforward to run
      // the intake at the desired speed. Store the individual values for logging.
      pidOutput = intakeController.calculate(getIntakeSpeed());
      newFeedforward = feedforward.calculate(intakeController.getSetpoint());
      intakeVoltageCommand = pidOutput + newFeedforward;

    } else {
      // If the intake isn't enabled, set the motor command to 0. In this state the intake
      // will slow down until it stops. Motor EMF braking will cause it to slow down faster
      // if that mode is used.
      pidOutput = 0;
      newFeedforward = 0;
      intakeVoltageCommand = 0;
    }
    intakeMotor.setVoltage(intakeVoltageCommand);
  }

  /** Returns a Command that runs the motor forward at the current set speed. */
  public Command runForward() {
    return new FunctionalCommand(
        this::setMotorSetPointForward,
        this::updateMotorController,
        interrupted -> disableIntake(),
        () -> false,
        this);
  }

  /** Returns a Command that runs the motor forward until a note is loaded. */
  public Command loadNote() {
    return new FunctionalCommand(
        this::setMotorSetPointForward,
        this::updateMotorController,
        interrupted -> disableIntake(),
        this::noteFullyLoaded,
        this);
  }

  /** Returns a Command that runs the motor in reverse at the current set speed. */
  public Command runReverse() {
    return new FunctionalCommand(
        this::setMotorSetPointReverse,
        this::updateMotorController,
        interrupted -> disableIntake(),
        () -> false,
        this);
  }

  /**
   * Set the setpoint for the motor as a scale factor applied to the setpoint value going forwards.
   * The PIDController drives the motor to this speed and holds it there.
   */
  private void setMotorSetPointForward() {
    loadTunableNumbers();
    intakeController.setSetpoint(forwardSetSpeed.get());

    // Call enable() to configure and start the controller in case it is not already enabled.
    enableIntake();
  }

  /**
   * Set the setpoint for the motor as a scale factor applied to the setpoint value reversed. The
   * PIDController drives the motor to this speed and holds it there.
   */
  private void setMotorSetPointReverse() {
    loadTunableNumbers();
    intakeController.setSetpoint(reverseSetSpeed.get());

    // Call enable() to configure and start the controller in case it is not already enabled.
    enableIntake();
  }

  /** Returns whether the intake has reached the set point speed within limits. */
  public boolean intakeAtSetpoint() {
    return intakeController.atSetpoint();
  }

  /**
   * Returns whether the intake has fully pulled in a note which is detected by the motor being up
   * to speed and the current hitting a threshold.
   */
  public boolean noteFullyLoaded() {
    return ((Math.abs(intakeEncoder.getVelocity()) >= speedThreshold.get())
        && (intakeMotor.getOutputCurrent() > currentThreshold.get()));
  }

  /**
   * Sets up the PID controller to run the intake at the defined setpoint speed. Preferences for
   * tuning the controller are applied.
   */
  private void enableIntake() {

    // Don't enable if already enabled since this may cause control transients
    if (!intakeEnabled) {
      loadTunableNumbers();

      // Reset the PID controller to clear any previous state
      intakeController.reset();
      intakeEnabled = true;

      DataLogManager.log(
          "Intake Enabled - kP="
              + intakeController.getP()
              + " kI="
              + intakeController.getI()
              + " kD="
              + intakeController.getD()
              + " Setpoint="
              + intakeController.getSetpoint()
              + " CurSpeed="
              + getIntakeSpeed());
    }
  }

  /**
   * Disables the PID control of the intake. Sets motor output to zero. NOTE: In this state the
   * intake will slow down until it stops. Motor EMF braking will cause it to slow down faster if
   * that mode is used.
   */
  public void disableIntake() {

    // Clear the enabled flag and update the controller to zero the motor command
    intakeEnabled = false;
    updateMotorController();

    DataLogManager.log("Intake Disabled CurSpeed=" + getIntakeSpeed());
  }

  /** Returns the intake speed for PID control and logging (Units are RPM). */
  public double getIntakeSpeed() {
    return intakeEncoder.getVelocity();
  }

  /** Returns the intake motor commanded voltage. */
  public double getIntakeVoltageCommand() {
    return intakeVoltageCommand;
  }

  /** Returns the motor for simulation. */
  public SparkMax getMotor() {
    return intakeMotor;
  }

  /**
   * Load values that can be tuned at runtime. This should only be called when the controller is
   * disabled - for example from enable().
   */
  private void loadTunableNumbers() {

    // Read values for PID controller
    intakeController.setP(kp.get());

    // Read values for Feedforward and create a new instance
    feedforward = new SimpleMotorFeedforward(ks.get(), kv.get(), 0.0);
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    intakeMotor.close();
  }
}
