// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.RollerSubsystem;
import frc.sim.Constants.RollerSimConstants;

/** A simulation for a simple DC motor with a load. */
public class RollerModel implements AutoCloseable {

  private final RollerSubsystem rollerSubsystem;
  private double simRollerCurrent = 0.0;
  private SparkMaxSim sparkSim;

  // The Roller driven by one motor.
  private final DCMotor motors = DCMotor.getNEO(1);

  private final LinearSystem<N2, N1, N2> plant =
      LinearSystemId.createDCMotorSystem(
          motors, RollerSimConstants.ROLLER_MOI_KG_METERS2, RollerConstants.ROLLER_GEAR_RATIO);

  private final DCMotorSim rollerMotorSim = new DCMotorSim(plant, motors);

  /**
   * Create a new RollerModel.
   *
   * @param rollerSubsystemToSimulate the RollerSubsystem to simulate
   */
  public RollerModel(RollerSubsystem rollerSubsystemToSimulate) {

    rollerSubsystem = rollerSubsystemToSimulate;
    simulationInit();

    // There is nothing to add to the dashboard for this sim since output is motor speed.
  }

  /** Initialize the claw simulation. */
  public void simulationInit() {

    // Setup a simulation of the SparkMax and methods to set values
    sparkSim = new SparkMaxSim(rollerSubsystem.getMotor(), motors);
  }

  /** Update the simulation model. */
  public void updateSim() {

    rollerMotorSim.setInput(rollerSubsystem.getRollerVoltageCommand());

    // Next, we update it. The standard loop time is 20ms.
    rollerMotorSim.update(0.020);

    // Finally, we run the spark simulation and save the current so it can be
    // retrieved later.
    sparkSim.iterate(rollerMotorSim.getAngularVelocityRPM(), 12.0, 0.02);
    simRollerCurrent =
        Math.abs(
            motors.getCurrent(
                rollerMotorSim.getAngularVelocityRadPerSec(),
                rollerSubsystem.getRollerVoltageCommand()));

    // Reset the simulation position if the subsystem has reset position
    if (rollerSubsystem.getResetSimPosition()) {
      sparkSim.setPosition(0.0);
      rollerSubsystem.clearResetSimPosition();
    }
  }

  /**
   * Return the simulated current.
   *
   * @return the simulated roller motor current in amperes
   */
  public double getSimCurrent() {
    return simRollerCurrent;
  }

  @Override
  public void close() {
    // Add closeable objects here
  }
}
