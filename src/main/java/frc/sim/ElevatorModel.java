// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.sim.Constants.ElevatorSimConstants;

/** A robot arm simulation based on a linear system model with Mech2d display. */
public class ElevatorModel implements AutoCloseable {

  private final ElevatorSubsystem elevatorSubsystem;
  private double simCurrent = 0.0;
  private SparkMaxSim sparkSim;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor elevatorGearbox = DCMotor.getNEO(1);

  // Simulation classes help us simulate what's going on, including gravity.
  // This elevator sim represents an elevator that can travel up and down when driven by the motor
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
  private final Mechanism2d mech2d = new Mechanism2d(1, 1.5);
  private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 0.5, 0.5);
  private final MechanismLigament2d elevatorMech2d =
      mech2dRoot.append(new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

  /** Create a new ElevatorModel. */
  public ElevatorModel(ElevatorSubsystem elevatorSubsystemToSimulate) {

    elevatorSubsystem = elevatorSubsystemToSimulate;
    simulationInit();

    // Put Mechanism 2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", mech2d);
  }

  /** Initialize the arm simulation. */
  public void simulationInit() {

    // Setup a simulation of the SparkMax and methods to set values
    sparkSim = new SparkMaxSim(elevatorSubsystem.getMotor(), elevatorGearbox);
  }

  /** Update the simulation model. */
  public void updateSim() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "input" (voltage)
    elevatorSim.setInput(elevatorSubsystem.getVoltageCommand());

    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.020);

    // Finally, we  run the spark simulations, set our simulated encoder's readings and save the
    // current so it can be retrieved later.
    sparkSim.iterate(elevatorSim.getVelocityMetersPerSecond(), 12.0, 0.02);
    simCurrent = elevatorSim.getCurrentDrawAmps();
    SmartDashboard.putNumber(
        "Elev Sim Torque (in-lbs)", elevatorGearbox.getTorque(simCurrent) * 8.85); // sim

    // Update elevator visualization with position
    elevatorMech2d.setLength(elevatorSim.getPositionMeters());
    updateShuffleboard();
  }

  /** Return the simulated current. */
  public double getSimCurrent() {
    return simCurrent;
  }

  public void updateShuffleboard() {

    SmartDashboard.putNumber("Elevator Sim Pos", elevatorSim.getPositionMeters()); // sim
  }

  @Override
  public void close() {
    mech2d.close();
    elevatorMech2d.close();
  }
}
