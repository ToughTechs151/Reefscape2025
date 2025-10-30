// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Adapted from FRC 4915 Spartronics https://github.com/Spartronics4915/2025-Reefscape

package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * A command that uses PID controllers to drive the swerve robot to a target pose (position and
 * rotation).
 *
 * <p>This command utilizes a {@link PPHolonomicDriveController} to calculate the required chassis
 * speeds to reach the goal pose. It monitors translation error, rotation error, and robot speed to
 * determine completion. The command ends when the robot is within specified position and rotation
 * tolerances, moving slowly enough, and this condition is debounced to prevent premature
 * termination.
 *
 * <p>The command includes logging for diagnostic purposes and stops the robot motors upon
 * completion or interruption.
 *
 * @see PPHolonomicDriveController
 * @see SwerveSubsystem
 */
public class PositionPIDCommand extends Command {

  private SwerveSubsystem swerve;

  /** The target pose (position and rotation) that the robot should drive to. */
  public final Pose2d goalPose;

  private static final String LOGGING_TABLE = "logging";

  private PPHolonomicDriveController driveController =
      new PPHolonomicDriveController(DriveConstants.TRANSLATION_PID, DriveConstants.ROTATION_PID);

  private final Trigger endTrigger;
  private final Trigger endTriggerDebounced;

  private final Timer timer = new Timer();

  private final BooleanPublisher endTriggerLogger =
      NetworkTableInstance.getDefault()
          .getTable(LOGGING_TABLE)
          .getBooleanTopic("PositionPIDEndTrigger")
          .publish();
  private final DoublePublisher xerrLogger =
      NetworkTableInstance.getDefault().getTable(LOGGING_TABLE).getDoubleTopic("X Error").publish();
  private final DoublePublisher yerrLogger =
      NetworkTableInstance.getDefault().getTable(LOGGING_TABLE).getDoubleTopic("Y Error").publish();

  private PositionPIDCommand(SwerveSubsystem swerve, Pose2d goalPose) {
    this.swerve = swerve;
    this.goalPose = goalPose;

    endTrigger =
        new Trigger(
            () -> {
              Pose2d diff = swerve.getPose().relativeTo(goalPose);

              var rotation =
                  MathUtil.isNear(
                      0.0,
                      diff.getRotation().getRotations(),
                      DriveConstants.ROTATION_TOLERANCE.getRotations(),
                      0.0,
                      1.0);

              var position =
                  diff.getTranslation().getNorm() < DriveConstants.POSITION_TOLERANCE.in(Meters);

              var speed = swerve.getSpeed() < DriveConstants.SPEED_TOLERANCE.in(MetersPerSecond);

              return rotation && position && speed;
            });

    endTriggerDebounced = endTrigger.debounce(DriveConstants.END_TRIGGER_DEBOUNCE.in(Seconds));
  }

  /**
   * Generates a command to drive the robot to a target pose using PID control, with a specified
   * timeout and stopping behavior.
   *
   * @param swerve the SwerveSubsystem instance used for driving
   * @param goalPose the target pose (position and rotation) to reach
   * @param timeout the maximum time allowed for the command to complete
   * @return a configured Command that drives to the goal pose, times out after the specified
   *     duration, and stops the robot upon completion
   */
  public static Command generateCommand(SwerveSubsystem swerve, Pose2d goalPose, Time timeout) {
    return new PositionPIDCommand(swerve, goalPose)
        .withTimeout(timeout)
        .finallyDo(
            () -> {
              swerve.drive(new ChassisSpeeds(0, 0, 0));
              swerve.lock();
            });
  }

  @Override
  public void initialize() {
    endTriggerLogger.accept(endTrigger.getAsBoolean());
    timer.restart();
  }

  @Override
  public void execute() {
    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    goalState.pose = goalPose;

    endTriggerLogger.accept(endTrigger.getAsBoolean());

    swerve.drive(driveController.calculateRobotRelativeSpeeds(swerve.getPose(), goalState));

    xerrLogger.accept(swerve.getPose().getX() - goalPose.getX());
    yerrLogger.accept(swerve.getPose().getY() - goalPose.getY());
  }

  @Override
  public void end(boolean interrupted) {
    endTriggerLogger.accept(endTrigger.getAsBoolean());
    timer.stop();

    Pose2d diff = swerve.getPose().relativeTo(goalPose);

    System.out.println(
        "Adjustments to alignment took: "
            + timer.get()
            + " seconds and interrupted = "
            + interrupted
            + "\nPosition offset: "
            + Centimeter.convertFrom(diff.getTranslation().getNorm(), Meters)
            + " cm"
            + "\nRotation offset: "
            + diff.getRotation().getMeasure().in(Degrees)
            + " deg"
            + "\nVelocity value: "
            + swerve.getSpeed()
            + "m/s");
  }

  @Override
  public boolean isFinished() {
    return endTriggerDebounced.getAsBoolean();
  }
}
