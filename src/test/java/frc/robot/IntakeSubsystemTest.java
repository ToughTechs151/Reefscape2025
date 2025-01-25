// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.AdditionalMatchers;

class IntakeSubsystemTest {
  private static final double DELTA = 5e-3;
  private Map<String, Double> telemetryDoubleMap = new HashMap<>();
  private Map<String, Boolean> telemetryBooleanMap = new HashMap<>();

  private IntakeSubsystem.Hardware intakeHardware;
  private IntakeSubsystem intake;
  private SparkMax mockMotor;
  private RelativeEncoder mockEncoder;

  @BeforeEach
  public void initEach() {
    // Create mock hardware devices
    mockMotor = mock(SparkMax.class);
    mockEncoder = mock(RelativeEncoder.class);

    // Create subsystem object using mock hardware
    intakeHardware = new IntakeSubsystem.Hardware(mockMotor, mockEncoder);
    intake = new IntakeSubsystem(intakeHardware);
  }

  @AfterEach
  public void closeIntake() {
    intake.close(); // motor is closed from the intake close method
  }

  @Test
  @DisplayName("Test constructor and initialization.")
  void testConstructor() {
    // We haven't enabled it yet, so command to motor and saved value should be zero.
    verify(mockMotor).setVoltage(0.0);
    assertThat(intake.getIntakeVoltageCommand()).isZero();
  }

  @Test
  @DisplayName("Test run forward command and disable.")
  void testForwardCommand() {

    // Create a command to run the intake then initialize
    Command runForwardCommand = intake.runForward();
    runForwardCommand.initialize();

    // Run the periodic method to generate telemetry and verify it was published
    intake.periodic();
    int numEntries = readTelemetry();
    assertThat(numEntries).isPositive();
    System.out.println("set point: " + telemetryDoubleMap.get("Intake Setpoint"));
    assertEquals(
        IntakeConstants.INTAKE_SET_POINT_FORWARD_RPM,
        telemetryDoubleMap.get("Intake Setpoint"),
        DELTA);

    // Execute the command to run the controller
    runForwardCommand.execute();
    intake.periodic();
    readTelemetry();
    assertThat(telemetryDoubleMap.get("Intake Voltage")).isPositive();
    assertThat(telemetryBooleanMap.get("Intake Enabled")).isTrue();

    // When disabled mMotor should be commanded to zero
    intake.disableIntake();
    intake.periodic();
    readTelemetry();
    verify(mockMotor, times(2)).setVoltage(0.0);
    assertThat(telemetryDoubleMap.get("Intake Voltage")).isZero();
    assertThat(telemetryBooleanMap.get("Intake Enabled")).isFalse();
  }

  @Test
  @DisplayName("Test run in reverse command.")
  void testReverseCommand() {

    // Create a command to run the intake then initialize
    Command runIntakeCommand = intake.runReverse();
    runIntakeCommand.initialize();

    // Run the periodic method to generate telemetry and verify it was published
    intake.periodic();
    int numEntries = readTelemetry();
    assertThat(numEntries).isPositive();
    assertEquals(
        IntakeConstants.INTAKE_SET_POINT_REVERSE_RPM,
        telemetryDoubleMap.get("Intake Setpoint"),
        DELTA);

    // Execute the command to run the controller
    runIntakeCommand.execute();
    intake.periodic();
    readTelemetry();
    assertThat(telemetryDoubleMap.get("Intake Voltage")).isNegative();
    assertThat(telemetryBooleanMap.get("Intake Enabled")).isTrue();
  }

  @Test
  @DisplayName("Test Motor and Encoder Sensors.")
  void testSensors() {

    // Set values for mocked sensors
    final double fakeCurrent = 3.3;
    when(mockMotor.getOutputCurrent()).thenReturn(fakeCurrent);
    final double fakeVelocity = 123.5;
    when(mockEncoder.getVelocity()).thenReturn(fakeVelocity);

    // The motor voltage should be set twice: once to 0 when configured and once to a
    // positive value when controller is run.
    Command runIntakeCommand = intake.runForward();
    runIntakeCommand.initialize();
    runIntakeCommand.execute();
    verify(mockMotor, times(2)).setVoltage(anyDouble());
    verify(mockMotor).setVoltage(0.0);
    verify(mockMotor, times(1)).setVoltage(AdditionalMatchers.gt(0.0));

    // Check that telemetry was sent to dashboard
    intake.periodic();
    readTelemetry();
    assertEquals(fakeCurrent, telemetryDoubleMap.get("Intake Current"), DELTA);
    assertEquals(fakeVelocity, telemetryDoubleMap.get("Intake Speed"), DELTA);
  }

  // ---------- Utility Functions --------------------------------------

  /* Read in telemetry values from the network table and store in maps */
  private int readTelemetry() {
    NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    Set<String> telemetryKeys = telemetryTable.getKeys();

    for (String keyName : telemetryKeys) {
      NetworkTableType entryType = telemetryTable.getEntry(keyName).getType();

      if (entryType == NetworkTableType.kDouble) {
        telemetryDoubleMap.put(keyName, telemetryTable.getEntry(keyName).getDouble(-1));
      } else if (entryType == NetworkTableType.kBoolean) {
        telemetryBooleanMap.put(keyName, telemetryTable.getEntry(keyName).getBoolean(false));
      }
    }
    return telemetryKeys.size();
  }
}
