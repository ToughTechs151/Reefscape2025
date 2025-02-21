// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.sim.Constants.ClawSim;
import frc.sim.Constants.ElevatorSimConstants;

/** A robot claw simulation based on a linear system model with Mech2d display. */
public class ElevatorModel implements AutoCloseable {

  private final ClawSubsystem clawSubsystem;
  private double simClawCurrent = 0.0;
  private SparkMaxSim sparkSimClaw;

  // The claw gearbox represents a gearbox containing one motor.
  private final DCMotor clawGearbox = DCMotor.getNEO(1);

  // This claw sim represents an claw that can rotate over the given mechanical range when driven
  // by the motor under the effect of gravity.
  private final SingleJointedArmSim clawSim =
      new SingleJointedArmSim(
          clawGearbox,
          ClawConstants.GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(ClawSim.CLAW_LENGTH_METERS, ClawSim.CLAW_MASS_KG),
          ClawSim.CLAW_LENGTH_METERS,
          ClawConstants.MIN_ANGLE_RADS,
          ClawConstants.MAX_ANGLE_RADS,
          true,
          ClawSim.START_ANGLE_RADS,
          ClawSim.ENCODER_DISTANCE_PER_PULSE,
          0.0 // Add noise with a std-dev of 1 tick
          );

  private final ElevatorSubsystem elevatorSubsystem;
  private double simElevatorCurrent = 0.0;
  private SparkMaxSim sparkSimElevator;
  private SparkAbsoluteEncoderSim absoluteEncoderSim;

  // The elevator gearbox represents a gearbox containing one motor.
  private final DCMotor elevatorGearbox = DCMotor.getNEO(1);

  // This elevator sim represents an elevator that can travel up and down when driven by the motor
  // under the effect of gravity. Simulation is for the first movable stage, so distances are
  // halved for the cascade.
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          elevatorGearbox,
          ElevatorConstants.GEAR_RATIO,
          ElevatorSimConstants.EFFECTIVE_MASS,
          ElevatorConstants.SPOOL_DIAMETER,
          ElevatorConstants.ELEVATOR_MIN_HEIGHT_METERS / 2.0,
          ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS / 2.0,
          true,
          0,
          0.002,
          0);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d mech2d = new Mechanism2d(1, 2);
  private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 0.5, 0.0);
  private final MechanismLigament2d baseMech2d =
      mech2dRoot.append(new MechanismLigament2d("Base", 0.2, 90, 20, new Color8Bit(Color.kGray)));
  private final MechanismLigament2d elevatorMech2d =
      baseMech2d.append(new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 0));

  // Attach moving Claw to the top of the elevator.
  private final MechanismLigament2d clawMech2d =
      elevatorMech2d.append(
          new MechanismLigament2d(
              "Claw",
              ClawSim.CLAW_LENGTH_METERS,
              Units.radiansToDegrees(clawSim.getAngleRads()) - 90,
              6,
              new Color8Bit(Color.kYellow)));

  /** Create a new ElevatorModel including the movable claw. */
  public ElevatorModel(
      ElevatorSubsystem elevatorSubsystemToSimulate, ClawSubsystem clawSubsystemToSimulate) {

    clawSubsystem = clawSubsystemToSimulate;
    elevatorSubsystem = elevatorSubsystemToSimulate;
    simulationInit();

    // Put Mechanism 2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", mech2d);
  }

  /** Initialize the elevator/claw simulation. */
  public void simulationInit() {

    // Setup a simulation of the SparkMax motors and methods to set values
    sparkSimElevator = new SparkMaxSim(elevatorSubsystem.getMotor(), elevatorGearbox);
    sparkSimClaw = new SparkMaxSim(clawSubsystem.getMotor(), clawGearbox);
    absoluteEncoderSim = sparkSimClaw.getAbsoluteEncoderSim();
  }

  /** Update the simulation model. */
  public void updateSim() {
    // In this method, we update our simulation of what our elevator and claw are doing
    // First, we set our "inputs" (voltages)
    elevatorSim.setInput(elevatorSubsystem.getVoltageCommand());
    clawSim.setInput(clawSubsystem.getVoltageCommand());

    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.020);
    clawSim.update(0.020);

    // Finally, we run the spark simulations and save the current so it can be retrieved later.
    // Double the simulated distances to account for the cascade elevator.
    sparkSimElevator.iterate(elevatorSim.getVelocityMetersPerSecond(), 12.0, 0.02);
    sparkSimClaw.iterate(clawSim.getVelocityRadPerSec(), 12.0, 0.02);
    sparkSimElevator.setPosition(
        2.0 * elevatorSim.getPositionMeters() - ElevatorConstants.ELEVATOR_OFFSET_METERS);
    sparkSimClaw.setPosition(clawSim.getAngleRads() - ClawConstants.CLAW_OFFSET_RADS);
    absoluteEncoderSim.setPosition(
        Units.radiansToRotations(clawSim.getAngleRads())
            + Units.degreesToRotations(ClawConstants.ABSOLUTE_OFFSET_DEGREES));

    // Update elevator/claw visualization with position (doubled) and angle
    elevatorMech2d.setLength(2.0 * elevatorSim.getPositionMeters());
    clawMech2d.setAngle(Units.radiansToDegrees(clawSim.getAngleRads()) - 90);
  }

  /** Return the simulated claw motor current. */
  public double getSimClawCurrent() {
    return simClawCurrent;
  }

  /** Return the simulated elevator motor current. */
  public double getSimElevatorCurrent() {
    return simElevatorCurrent;
  }

  @Override
  public void close() {
    mech2d.close();
    baseMech2d.close();
    clawMech2d.close();
    elevatorMech2d.close();
  }
}
