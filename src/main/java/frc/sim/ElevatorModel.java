// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.sim.Constants.ArmSim;
import frc.sim.Constants.ElevatorSimConstants;

/** A robot arm simulation based on a linear system model with Mech2d display. */
public class ElevatorModel implements AutoCloseable {

  private final ArmSubsystem armSubsystem;
  private double simArmCurrent = 0.0;
  private SparkMaxSim sparkSimArm;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor armGearbox = DCMotor.getNEO(1);

  // This arm sim represents an arm that can rotate over the given mechanical range when driven
  // by the motor under the effect of gravity.
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          armGearbox,
          ArmSim.ARM_REDUCTION,
          SingleJointedArmSim.estimateMOI(ArmSim.ARM_LENGTH_METERS, ArmSim.ARM_MASS_KG),
          ArmSim.ARM_LENGTH_METERS,
          ArmConstants.MIN_ANGLE_RADS,
          ArmConstants.MAX_ANGLE_RADS,
          true,
          ArmSim.START_ANGLE_RADS,
          ArmSim.ENCODER_DISTANCE_PER_PULSE,
          0.0 // Add noise with a std-dev of 1 tick
          );

  private final ElevatorSubsystem elevatorSubsystem;
  private double simElevatorCurrent = 0.0;
  private SparkMaxSim sparkSimElevator;

  // The elevator gearbox represents a gearbox containing one motor.
  private final DCMotor elevatorGearbox = DCMotor.getNEO(1);

  // This elevator sim represents an elevator that can travel up and down when driven by the motor
  // under the effect of gravity.
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          elevatorGearbox,
          ElevatorSimConstants.ELEVATOR_REDUCTION,
          ElevatorSimConstants.CARRIAGE_MASS,
          ElevatorSimConstants.ELEVATOR_DRUM_RADIUS,
          ElevatorConstants.ELEVATOR_MIN_HEIGHT_METERS,
          ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS,
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

  // Attach moving Arm to the top of the elevator.
  private final MechanismLigament2d armMech2d =
      elevatorMech2d.append(
          new MechanismLigament2d(
              "Arm",
              ArmSim.ARM_LENGTH_METERS,
              Units.radiansToDegrees(armSim.getAngleRads() - 90),
              6,
              new Color8Bit(Color.kYellow)));

  /** Create a new ElevatorModel including the movable arm. */
  public ElevatorModel(
      ElevatorSubsystem elevatorSubsystemToSimulate, ArmSubsystem armSubsystemToSimulate) {

    armSubsystem = armSubsystemToSimulate;
    elevatorSubsystem = elevatorSubsystemToSimulate;
    simulationInit();

    // Put Mechanism 2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", mech2d);
  }

  /** Initialize the elevator/arm simulation. */
  public void simulationInit() {

    // Setup a simulation of the SparkMax motors and methods to set values
    sparkSimElevator = new SparkMaxSim(elevatorSubsystem.getMotor(), elevatorGearbox);
    sparkSimArm = new SparkMaxSim(armSubsystem.getMotor(), armGearbox);
  }

  /** Update the simulation model. */
  public void updateSim() {
    // In this method, we update our simulation of what our elevator and arm are doing
    // First, we set our "inputs" (voltages)
    elevatorSim.setInput(elevatorSubsystem.getVoltageCommand());
    armSim.setInput(armSubsystem.getVoltageCommand());

    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.020);
    armSim.update(0.020);

    // Finally, we run the spark simulations and save the
    // current so it can be retrieved later.
    sparkSimElevator.iterate(elevatorSim.getVelocityMetersPerSecond(), 12.0, 0.02);
    sparkSimArm.iterate(armSim.getVelocityRadPerSec(), 12.0, 0.02);
    simElevatorCurrent = Math.abs(elevatorSim.getCurrentDrawAmps());
    simArmCurrent = Math.abs(armSim.getCurrentDrawAmps());

    // Update elevator/arm visualization with position and angle
    elevatorMech2d.setLength(elevatorSim.getPositionMeters());
    armMech2d.setAngle(Units.radiansToDegrees(armSim.getAngleRads()) - 90);
  }

  /** Return the simulated arm motor current. */
  public double getSimArmCurrent() {
    return simArmCurrent;
  }

  /** Return the simulated elevator motor current. */
  public double getSimElevatorCurrent() {
    return simElevatorCurrent;
  }

  @Override
  public void close() {
    mech2d.close();
    baseMech2d.close();
    armMech2d.close();
    elevatorMech2d.close();
  }
}
