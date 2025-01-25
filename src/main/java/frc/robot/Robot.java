// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.sim.RobotModel;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private DataLogging datalog;

  /**
   * {@code robotInit} runs when the robot first starts up. It is used to create the robot
   * container, and can optionally be used for any initialization code.
   */
  @Override
  public void robotInit() {

    // Initialize the data logging.
    datalog = DataLogging.getInstance();

    // Print our splash screen info.
    Splash.printAllStatusFiles();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    this.robotContainer = new RobotContainer();

    datalog.dataLogRobotContainerInit(this.robotContainer);
  }

  /**
   * {@code robotPeriodic} called every 20 ms, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
    // running already-scheduled commands, removing finished or interrupted commands, and running
    // subsystem periodic() methods. This must be called from the robot's periodic block in order
    // for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Must be at the end of robotPeriodic
    datalog.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

    if (isSimulation() && simModel != null) {
      simModel.reset();
    }

    CommandScheduler.getInstance().cancelAll();
    this.robotContainer.disableSubsystems();
  }

  @Override
  public void disabledPeriodic() {
    // Add code to run repeatedly while disabled.
    datalog.startLoopTime();
  }

  /**
   * {@code autonomousInit} runs when the robot enters autonomous mode. It is used to schedule the
   * desired autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    if (this.robotContainer == null) {
      DriverStation.reportError("autonomousInit called with null robotContainer", false);
    } else {

      // Cancel any commands already running.
      CommandScheduler.getInstance().cancelAll();

      this.autonomousCommand = this.robotContainer.getAutonomousCommand();

      // schedule the autonomous command (example)
      if (this.autonomousCommand != null) {
        this.autonomousCommand.schedule();
      }
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    datalog.startLoopTime();
    // Add code to run repeatedly during Autonomous mode.
  }

  /** This function is called once at the start of operator control. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (this.autonomousCommand != null) {
      this.autonomousCommand.cancel();
    }

    // Get the selected drive mode to use in TeleOp mode
    Command driveCommand = robotContainer.getTeleopDriveCommand();

    // schedule the drive command
    if (driveCommand != null) {
      driveCommand.schedule();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    datalog.startLoopTime();
    // Add code to run repeatedly during Teleop mode.
  }

  /** This function is called once at the start of test mode. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // Generally test mode will have the same Init and Periodic code as Teleop,
    // so call them here. Replace if desired.
    teleopInit();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // Generally test mode will have the same Init and Periodic code as Teleop,
    // so call them here. Replace if desired. If you don't call teleopPeriodic
    // here, then add a call to StartLoopTime.
    teleopPeriodic();
    // Add code to run repeatedly during Test mode.
  }

  /*
   * This set of functions is for simulation support only, and is not called on
   * the real robot. Put plant-model related functionality here. For training
   * purposes, students should not have to modify this functionality.
   */

  // Simple robot plant model for simulation purposes
  RobotModel simModel;

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

    // Add code to run when the robot is initialized during simulations.
    simModel = new RobotModel(this);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    // Add code to run repeatedly during simulations.
    if (isSimulation() && simModel != null) {
      simModel.update();
    }
  }

  public RobotContainer getRobotContainer() {
    return robotContainer;
  }
}
