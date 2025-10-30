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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.AdditionalMatchers;

class ElevatorSubsystemTest {
  private static final double DELTA = 5e-3;
  private Map<String, Double> telemetryDoubleMap = new HashMap<>();
  private Map<String, Boolean> telemetryBooleanMap = new HashMap<>();

  private ElevatorSubsystem.Hardware elevatorHardware;
  private ElevatorSubsystem elevator;
  private SparkMax mockMotor;
  private RelativeEncoder mockEncoder;

  @BeforeEach
  void initEach() {
    // Create mock hardware devices
    mockMotor = mock(SparkMax.class);
    mockEncoder = mock(RelativeEncoder.class);

    // Create subsystem object using mock hardware
    elevatorHardware = new ElevatorSubsystem.Hardware(mockMotor, mockEncoder);
    elevator = new ElevatorSubsystem(elevatorHardware);
  }

  @AfterEach
  void closeelevator() {
    elevator.close(); // motor is closed from the elevator close method
  }

  @Test
  @DisplayName("Test constructor and initialization.")
  void testConstructor() {
    // We haven't enabled it yet, so command to motor and saved value should be zero.
    verify(mockMotor).setVoltage(0.0);
    assertThat(elevator.getVoltageCommand()).isZero();

    // Position should be set to starting position
    assertThat(elevator.getMeasurement()).isEqualTo(ElevatorConstants.ELEVATOR_OFFSET_METERS);
  }

  @Test
  @DisplayName("Test move command and disable.")
  void testMoveCommand() {

    // Create a command to move the elevator then initialize
    Command moveCommand = elevator.moveToPosition(ElevatorConstants.ELEVATOR_LEVEL2);
    moveCommand.initialize();

    // Run the periodic method to generate telemetry and verify it was published
    elevator.periodic();
    int numEntries = readTelemetry();
    assertThat(numEntries).isPositive();
    assertEquals(ElevatorConstants.ELEVATOR_LEVEL2, telemetryDoubleMap.get("Goal"), DELTA);

    // Execute the command to run the controller
    moveCommand.execute();
    elevator.periodic();
    readTelemetry();
    assertThat(telemetryDoubleMap.get("Voltage")).isPositive();
    assertThat(telemetryBooleanMap.get("Enabled")).isTrue();

    // When disabled mMotor should be commanded to zero
    elevator.disable();
    elevator.periodic();
    readTelemetry();
    verify(mockMotor, times(2)).setVoltage(0.0);
    assertThat(telemetryDoubleMap.get("Voltage")).isZero();
    assertThat(telemetryBooleanMap.get("Enabled")).isFalse();
  }

  @Test
  @DisplayName("Test Motor and Encoder Sensors.")
  void testSensors() {

    // Set values for mocked sensors
    final double fakeCurrent = -3.3;
    when(mockMotor.getOutputCurrent()).thenReturn(fakeCurrent);
    final double fakePosition = 0.1;
    when(mockEncoder.getPosition()).thenReturn(fakePosition);
    final double fakeVelocity = 0.123;
    when(mockEncoder.getVelocity()).thenReturn(fakeVelocity);

    // The motor voltage should be set twice: once to 0 when configured and once  to a
    // positive value when controller is run.
    Command moveCommand = elevator.moveToPosition(Constants.ElevatorConstants.ELEVATOR_LEVEL2);
    moveCommand.initialize();
    System.out.println(elevator.getVoltageCommand());
    moveCommand.execute();
    System.out.println(elevator.getVoltageCommand());
    verify(mockMotor, times(2)).setVoltage(anyDouble());
    verify(mockMotor).setVoltage(0.0);
    verify(mockMotor, times(1)).setVoltage(AdditionalMatchers.gt(0.0));

    // Test position measurements from the encoder
    assertThat(elevator.getMeasurement())
        .isEqualTo(ElevatorConstants.ELEVATOR_OFFSET_METERS + fakePosition);

    // Check that telemetry was sent to dashboard
    elevator.periodic();
    readTelemetry();
    assertEquals(fakeCurrent, telemetryDoubleMap.get("Current"), DELTA);
    assertEquals(
        ElevatorConstants.ELEVATOR_OFFSET_METERS + fakePosition,
        telemetryDoubleMap.get("Position"),
        DELTA);
    assertEquals(fakeVelocity, telemetryDoubleMap.get("Velocity"), DELTA);
  }

  @Test
  @DisplayName("Test range limit and hold.")
  void testLimitAndHoldCommand() {

    // Try a command to move the elevator above the limit
    Command moveCommand =
        elevator.moveToPosition(Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS + 0.1);
    moveCommand.initialize();
    elevator.periodic();
    readTelemetry();
    assertEquals(
        ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS, telemetryDoubleMap.get("Goal"), DELTA);

    // Verify that the hold command runs the controller
    Command moveCommandHigh = elevator.moveToPosition(Constants.ElevatorConstants.ELEVATOR_LEVEL4);
    Command holdCommand = elevator.holdPosition();
    // Initialize to set goal but don't execute so hold can be checked
    moveCommandHigh.initialize();
    holdCommand.execute();
    elevator.periodic();
    readTelemetry();

    // Motor command should be positive to move elevator up.
    assertThat(telemetryDoubleMap.get("Voltage")).isPositive();
    assertThat(telemetryBooleanMap.get("Enabled")).isTrue();
  }

  // ---------- Utility Functions --------------------------------------

  /* Read in telemetry values from the network table and store in maps */
  private int readTelemetry() {
    NetworkTable telemetryTable =
        NetworkTableInstance.getDefault().getTable("SmartDashboard/Elevator");
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
