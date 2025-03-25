// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Adapted from https://github.com/frc5517/2025-Reefscape

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ProfileToPose extends Command {
  private static final double drivekP = 0.9;
  private static final double drivekD = 0.01;
  private static final double thetakP = 4.0;
  private static final double thetakD = 0.0;
  private static final double driveMaxVelocity = 0.7;
  private static final double driveMaxAcceleration = 1;
  private static final double thetaMaxVelocity = Units.degreesToRadians(180.0);
  private static final double thetaMaxAcceleration = Units.degreesToRadians(720.0);
  private static final double driveTolerance = 0.02; // meters
  private static final double thetaTolerance = Units.degreesToRadians(1.0);
  private static final double ffMinRadius = 0.02;
  private static final double ffMaxRadius = 0.08;

  private SwerveSubsystem swerve;
  private Pose2d target;

  private static final ProfiledPIDController driveController =
      new ProfiledPIDController(
          drivekP,
          0,
          drivekD,
          new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
  private static final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetakP,
          0,
          thetakD,
          new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));
  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;
  private Supplier<Pose2d> robot = () -> swerve.getPose();

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  public ProfileToPose(SwerveSubsystem swerve, Pose2d target) {
    this.swerve = swerve;
    this.target = target;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveController.setTolerance(driveTolerance);
    thetaController.setTolerance(thetaTolerance);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = robot.get();
    ChassisSpeeds fieldVelocity = swerve.getRobotVelocity();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    driveController.reset(
        currentPose.getTranslation().getDistance(target.getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    target
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    // Get current pose and target pose
    Pose2d currentPose = robot.get();
    Pose2d targetPose = target;

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) {
      driveVelocityScalar = 0.0;
    }
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                new Rotation2d(
                    Math.atan2(
                        currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                        currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
            .transformBy(
                new Transform2d(driveController.getSetpoint().position, 0.0, Rotation2d.kZero))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) {
      thetaVelocity = 0.0;
    }

    Translation2d driveVelocity =
        new Pose2d(
                Translation2d.kZero,
                new Rotation2d(
                    Math.atan2(
                        currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                        currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, Rotation2d.kZero))
            .getTranslation();

    // Scale feedback velocities by input ff
    final double linearS = linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    driveVelocity = driveVelocity.interpolate(linearFF.get().times(4.69), linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity, omegaFF.getAsDouble() * 4.69 / Math.hypot(10.75, 10.75), thetaS);

    // Command speeds (ROBOT RELATIVE)
    swerve.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
    SmartDashboard.putBoolean("PID at Goal", atGoal());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.lock();
    running = false;
  }

  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }
}
