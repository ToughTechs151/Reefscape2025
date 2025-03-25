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

public class PositionPIDCommand extends Command {

  public SwerveSubsystem mSwerve;
  public final Pose2d goalPose;
  private PPHolonomicDriveController mDriveController =
      new PPHolonomicDriveController(
          DriveConstants.KP_TRANSLATION_PID, DriveConstants.KP_ROTATION_PID);

  private final Trigger endTrigger;
  private final Trigger endTriggerDebounced;

  private final Timer timer = new Timer();

  private final BooleanPublisher endTriggerLogger =
      NetworkTableInstance.getDefault()
          .getTable("logging")
          .getBooleanTopic("PositionPIDEndTrigger")
          .publish();
  private final DoublePublisher xErrLogger =
      NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("X Error").publish();
  private final DoublePublisher yErrLogger =
      NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("Y Error").publish();

  private PositionPIDCommand(SwerveSubsystem mSwerve, Pose2d goalPose) {
    this.mSwerve = mSwerve;
    this.goalPose = goalPose;

    endTrigger =
        new Trigger(
            () -> {
              Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

              var rotation =
                  MathUtil.isNear(
                      0.0,
                      diff.getRotation().getRotations(),
                      DriveConstants.kRotationTolerance.getRotations(),
                      0.0,
                      1.0);

              var position =
                  diff.getTranslation().getNorm() < DriveConstants.kPositionTolerance.in(Meters);

              var speed = mSwerve.getSpeed() < DriveConstants.kSpeedTolerance.in(MetersPerSecond);

              // System.out.println("end trigger conditions R: "+ rotation + "\tP: " + position +
              // "\tS: " + speed);

              return rotation && position && speed;
            });

    endTriggerDebounced = endTrigger.debounce(DriveConstants.kEndTriggerDebounce.in(Seconds));
  }

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

    mSwerve.drive(mDriveController.calculateRobotRelativeSpeeds(mSwerve.getPose(), goalState));

    xErrLogger.accept(mSwerve.getPose().getX() - goalPose.getX());
    yErrLogger.accept(mSwerve.getPose().getY() - goalPose.getY());
  }

  @Override
  public void end(boolean interrupted) {
    endTriggerLogger.accept(endTrigger.getAsBoolean());
    timer.stop();

    Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

    System.out.println(
        "Adjustments to alginment took: "
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
            + mSwerve.getSpeed()
            + "m/s");
  }

  @Override
  public boolean isFinished() {
    return endTriggerDebounced.getAsBoolean();
  }
}
