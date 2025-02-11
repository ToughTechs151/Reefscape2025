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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.AdditionalMatchers;

class ClawSubsystemTest {
  private static final double DELTA = 5e-3;
  private Map<String, Double> telemetryDoubleMap = new HashMap<>();
  private Map<String, Boolean> telemetryBooleanMap = new HashMap<>();

  private ClawSubsystem.Hardware clawHardware;
  private ClawSubsystem claw;
  private SparkMax mockMotor;
  private RelativeEncoder mockEncoder;

  @BeforeEach
  public void initEach() {
    // Create mock hardware devices
    mockMotor = mock(SparkMax.class);
    mockEncoder = mock(RelativeEncoder.class);

    // Create subsystem object using mock hardware
    clawHardware = new ClawSubsystem.Hardware(mockMotor, mockEncoder);
    claw = new ClawSubsystem(clawHardware);
  }

  @AfterEach
  public void closeClaw() {
    claw.close(); // motor is closed from the claw close method
  }

  @Test
  @DisplayName("Test constructor and initialization.")
  void testConstructor() {
    // We haven't enabled it yet, so command to motor and saved value should be zero.
    verify(mockMotor).setVoltage(0.0);
    assertThat(claw.getVoltageCommand()).isZero();

    // Position should be set to starting position
    assertThat(claw.getMeasurement()).isEqualTo(ClawConstants.CLAW_OFFSET_RADS);
  }

  @Test
  @DisplayName("Test move command and disable.")
  void testMoveCommand() {

    // Create a command to move the claw then initialize
    Command moveCommand = claw.moveToPosition(Constants.ClawConstants.CLAW_LEVEL1_RADS);
    moveCommand.initialize();

    // Run the periodic method to generate telemetry and verify it was published
    claw.periodic();
    int numEntries = readTelemetry();
    assertThat(numEntries).isPositive();
    assertEquals(
        Units.radiansToDegrees(ClawConstants.CLAW_LEVEL1_RADS),
        telemetryDoubleMap.get("Claw Goal"),
        DELTA);

    // Execute the command to run the controller
    moveCommand.execute();
    claw.periodic();
    readTelemetry();
    assertThat(telemetryDoubleMap.get("Claw Voltage")).isPositive();
    assertThat(telemetryBooleanMap.get("Claw Enabled")).isTrue();

    // When disabled mMotor should be commanded to zero
    claw.disable();
    claw.periodic();
    readTelemetry();
    verify(mockMotor, times(2)).setVoltage(0.0);
    assertThat(telemetryDoubleMap.get("Claw Voltage")).isZero();
    assertThat(telemetryBooleanMap.get("Claw Enabled")).isFalse();
  }

  @Test
  @DisplayName("Test Motor and Encoder Sensors.")
  void testSensors() {

    // Set values for mocked sensors
    final double fakeCurrent = -3.3;
    when(mockMotor.getOutputCurrent()).thenReturn(fakeCurrent);
    final double fakePosition = 1.5;
    when(mockEncoder.getPosition()).thenReturn(fakePosition);
    final double fakeVelocity = 0.123;
    when(mockEncoder.getVelocity()).thenReturn(fakeVelocity);

    // The motor voltage should be set twice: once to 0 when configured and once  to a
    // negative value when controller is run.
    Command moveCommand = claw.moveToPosition(Constants.ClawConstants.CLAW_LEVEL2_AND_LEVEL3_RADS);
    moveCommand.initialize();
    moveCommand.execute();
    verify(mockMotor, times(2)).setVoltage(anyDouble());
    verify(mockMotor).setVoltage(0.0);
    verify(mockMotor, times(1)).setVoltage(AdditionalMatchers.lt(0.0));

    // This unused code is provided as an example of looking for a specific value.
    // This value was cheated by running working code as an example since calculating actual
    // controller expected values is difficult.  Instead the test above just checks direction
    // of the command, and controller response tests are done in simulation by checking desired
    // response over time.
    //
    // final double expectedCommand = 0.34092;
    // verify(mockMotor, times(1)).setVoltage(AdditionalMatchers.eq(expectedCommand, DELTA));

    // Alternative method: capture values and then use them in a test criteria
    // ArgumentCaptor<Double> argument = ArgumentCaptor.forClass(Double.class);
    // verify(mockMotor).setVoltage(argument.capture()); // Can use this if only called once
    // verify(mockMotor, times(2)).setVoltage(argument.capture());
    // assertEquals(expectedCommand, argument.getValue(), DELTA);

    // Test position measurements from the encoder
    assertThat(claw.getMeasurement()).isEqualTo(ClawConstants.CLAW_OFFSET_RADS + fakePosition);

    // Check that telemetry was sent to dashboard
    claw.periodic();
    readTelemetry();
    assertEquals(fakeCurrent, telemetryDoubleMap.get("Claw Current"), DELTA);
    assertEquals(
        Units.radiansToDegrees(ClawConstants.CLAW_OFFSET_RADS + fakePosition),
        telemetryDoubleMap.get("Claw Angle"),
        DELTA);
    if (Constants.SD_SHOW_CLAW_EXTENDED_LOGGING_DATA) {
      assertEquals(
          Units.radiansToDegrees(fakeVelocity), telemetryDoubleMap.get("Claw Velocity"), DELTA);
    }
  }

  @Test
  @DisplayName("Test range limit and hold.")
  void testLimitAndHoldCommand() {

    // Try a command to move the claw above the limit
    Command moveCommand = claw.moveToPosition(Constants.ClawConstants.MAX_ANGLE_RADS + 0.1);
    moveCommand.initialize();
    claw.periodic();
    readTelemetry();
    assertEquals(
        Units.radiansToDegrees(ClawConstants.MAX_ANGLE_RADS),
        telemetryDoubleMap.get("Claw Goal"),
        DELTA);

    // Verify that the hold command runs the controller
    Command moveCommandHigh = claw.moveToPosition(Constants.ClawConstants.CLAW_ALGAE_RADS);
    Command holdCommand = claw.holdPosition();
    // Initialize to set goal but don't execute so hold can be checked
    moveCommandHigh.initialize();
    holdCommand.execute();
    claw.periodic();
    readTelemetry();

    // Motor command should be negative to hold claw up.
    assertThat(telemetryDoubleMap.get("Claw Voltage")).isPositive();
    assertThat(telemetryBooleanMap.get("Claw Enabled")).isTrue();
  }

  @Test
  @DisplayName("Test shift down and up commands.")
  void testShiftDownCommand() {
    Command moveCommand = claw.moveToPosition(Constants.ClawConstants.CLAW_LEVEL4_RADS);
    Command upCommand = claw.shiftUp();

    // Command to a position and then shift up
    moveCommand.initialize();
    upCommand.initialize();
    claw.periodic();
    readTelemetry();
    assertEquals(
        Units.radiansToDegrees(ClawConstants.CLAW_LEVEL4_RADS + ClawConstants.POS_INCREMENT),
        telemetryDoubleMap.get("Claw Goal"),
        DELTA);

    // Shift down
    Command downCommand = claw.shiftDown();
    downCommand.initialize();
    claw.periodic();
    readTelemetry();
    // Currently up and down increments are the same. Update if that changes.
    assertEquals(
        Units.radiansToDegrees(
            ClawConstants.CLAW_LEVEL4_RADS
                + ClawConstants.POS_INCREMENT
                - ClawConstants.POS_INCREMENT),
        telemetryDoubleMap.get("Claw Goal"),
        DELTA);
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

  // *** Available Telemetry Keys ***
  // "Claw Enabled"
  // "Claw Goal"
  // "Claw Angle"
  // "Claw Velocity"
  // "Claw Voltage"
  // "Claw Current"
  // "Claw Feedforward"
  // "Claw PID output"
  // "Claw SetPt Pos"
  // "Claw SetPt Vel"
