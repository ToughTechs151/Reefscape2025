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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.sim.Constants.IntakeSimConstants;

/** A simulation for a simple DC motor with a load. */
public class IntakeModel implements AutoCloseable {

  private final IntakeSubsystem intakeSubsystem;
  private double simIntakeCurrent = 0.0;
  private SparkMaxSim sparkSim;

  // The intake driven by one motor.
  private final DCMotor motors = DCMotor.getNEO(1);

  private final LinearSystem<N2, N1, N2> plant =
      LinearSystemId.createDCMotorSystem(
          motors, IntakeSimConstants.INTAKE_MOI_KG_METERS2, IntakeConstants.INTAKE_GEAR_RATIO);

  private final DCMotorSim intakeMotorSim = new DCMotorSim(plant, motors);

  /** Create a new ElevatorModel. */
  public IntakeModel(IntakeSubsystem intakeSubsystemToSimulate) {

    intakeSubsystem = intakeSubsystemToSimulate;
    simulationInit();

    // There is nothing to add to the dashboard for this sim since output is motor speed.
  }

  /** Initialize the arm simulation. */
  public void simulationInit() {

    // Setup a simulation of the SparkMax and methods to set values
    sparkSim = new SparkMaxSim(intakeSubsystem.getMotor(), motors);
  }

  /** Update the simulation model. */
  public void updateSim() {

    intakeMotorSim.setInput(intakeSubsystem.getIntakeVoltageCommand());

    // Next, we update it. The standard loop time is 20ms.
    intakeMotorSim.update(0.020);

    // Finally, we run the spark simulation and save the current so it can be
    // retrieved later.
    sparkSim.iterate(intakeMotorSim.getAngularVelocityRPM(), 12.0, 0.02);
    SmartDashboard.putNumber("Sim Intake input", intakeMotorSim.getAngularVelocityRPM());
    simIntakeCurrent =
        Math.abs(
            motors.getCurrent(
                intakeMotorSim.getAngularVelocityRadPerSec(),
                intakeSubsystem.getIntakeVoltageCommand()));
  }

  /** Return the simulated current. */
  public double getSimCurrent() {
    return simIntakeCurrent;
  }

  @Override
  public void close() {
    // Add closeable objects here
  }
}
