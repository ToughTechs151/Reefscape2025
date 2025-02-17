package frc.sim;

/* PDP sim code poached from https://github.com/RobotCasserole1736/TheBestSwerve2021 */

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Robot;
import java.util.Random;

public class RobotModel {

  PDPSim simpdp;

  // Mechanical elevator driven by motor with gear reduction with attached mechanical claw
  // driven by motor with gear reduction for simulation purposes.
  // Works in conjunction with ElevatorSubsystem and ClawSubsystem
  ElevatorModel simElevator;

  // Mechanical Roller driven by motor with gear reduction for simulation purposes.
  // Works in conjunction with RollerSubsystem
  RollerModel simRoller;

  Random random = new Random();
  private final boolean isReal;
  static final double QUIESCENT_CURRENT_DRAW_A = 2.0; // Misc electronics
  static final double BATTERY_NOMINAL_VOLTAGE = 13.2; // Nicely charged battery
  static final double BATTERY_NOMINAL_RESISTANCE = 0.010; // average battery + cabling
  double currentDrawA = QUIESCENT_CURRENT_DRAW_A;
  double batteryVoltageV = BATTERY_NOMINAL_VOLTAGE;

  /**
   * Create robot simulation. Does nothing if not running a simulation. Called from Robot.java as a
   * class field.
   *
   * @param robot Robot
   */
  public RobotModel(Robot robot) {
    if (RobotBase.isSimulation()) {
      isReal = false;
    } else {
      isReal = true;
      return;
    }

    simElevator =
        new ElevatorModel(
            robot.getRobotContainer().getElevatorSubsystem(),
            robot.getRobotContainer().getClawSubsystem());

    simRoller = new RollerModel(robot.getRobotContainer().getRollerSubsystem());

    simpdp = new PDPSim(robot.getRobotContainer().getPdp());
    reset();
  }

  /** Update the simulation model. Call from simulationPeriodic method in robot.java. */
  public void update() {
    if (isReal) {
      return;
    }

    // Update subsystem simulations
    simElevator.updateSim();
    simRoller.updateSim();

    // Simulate battery voltage drop based on total simulated current
    double clawCurrent = Math.abs(simElevator.getSimClawCurrent());
    double elevatorCurrent = Math.abs(simElevator.getSimElevatorCurrent());
    double rollerCurrent = Math.abs(simRoller.getSimCurrent());

    double[] simCurrents = {clawCurrent, elevatorCurrent, rollerCurrent};

    double unloadedVoltage = batteryVoltageV * 0.98 + ((random.nextDouble() / 10) - 0.05);
    double loadedVoltage =
        BatterySim.calculateLoadedBatteryVoltage(
            unloadedVoltage, BATTERY_NOMINAL_RESISTANCE, simCurrents);
    RoboRioSim.setVInVoltage(loadedVoltage);

    simpdp.setVoltage(loadedVoltage);
    simpdp.setCurrent(0, currentDrawA + random.nextDouble());
    simpdp.setCurrent(1, clawCurrent);
    simpdp.setCurrent(5, elevatorCurrent);
    simpdp.setCurrent(8, rollerCurrent);
    simpdp.setTemperature(26.5);
  }

  /** Reset the simulation data. */
  public final void reset() {
    if (isReal) {
      return;
    }
    simpdp.resetData();
    RoboRioSim.resetData();
  }
}
