// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Set;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // First we do things that are in all Robots.
  private PowerDistribution pdp = new PowerDistribution();
  // The driver's controller
  private CommandXboxController driverController =
      new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  // The operator's controller
  private CommandXboxController operatorController =
      new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

  // Now all the subsystems.
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final LEDSubsystem led = new LEDSubsystem();

  private final ClawSubsystem robotClaw = new ClawSubsystem(ClawSubsystem.initializeHardware());

  private final ElevatorSubsystem robotElevator =
      new ElevatorSubsystem(ElevatorSubsystem.initializeHardware());

  private final RollerSubsystem robotRoller =
      new RollerSubsystem(RollerSubsystem.initializeHardware());

  private final Trigger unsafeTrigger = new Trigger(() -> !isSafePosition());
  private final Trigger safeTrigger = new Trigger(() -> isSafePosition());

  // Commands to drive to the closest face of the reef offset to left or right.
  private Command driveToClosestReefLeft =
      new DeferredCommand(
              () -> createDriveReefCommand(FieldConstants.REEF_SHIFT_LEFT), Set.of(drivebase))
          .withName("Drive to Reef Left");
  private Command driveToClosestReefRight =
      new DeferredCommand(
              () -> createDriveReefCommand(FieldConstants.REEF_SHIFT_RIGHT), Set.of(drivebase))
          .withName("Drive to Reef Right");

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverController.getLeftY() * -1,
              () -> driverController.getLeftX() * -1)
          .withControllerRotationAxis(() -> driverController.getRightX() * -1)
          .deadband(OIConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(DriveConstants.USE_ALLIANCE);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAngularVelocity =
      drivebase.driveFieldOriented(driveAngularVelocity).withName("Angular Velocity");

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  // This doesn't do what we want in 2025.1.1
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveRobotOrientedAngularVelocity =
      drivebase.driveFieldOriented(driveRobotOriented).withName("Robot Oriented");

  // Commands to shift robot position at low speed using POV
  SwerveInputStream shiftForwardRobotOriented =
      SwerveInputStream.of(drivebase.getSwerveDrive(), () -> DriveConstants.POV_SPEED, () -> 0.0)
          .withControllerRotationAxis(() -> 0.0)
          .robotRelative(true)
          .allianceRelativeControl(false);

  Command shiftForward =
      drivebase.driveFieldOriented(shiftForwardRobotOriented).withName("Shift Forward");

  // Commands to shift robot position at low speed using POV
  SwerveInputStream shiftBackRobotOriented =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(), () -> -1 * DriveConstants.POV_SPEED, () -> 0.0)
          .withControllerRotationAxis(() -> 0.0)
          .robotRelative(true)
          .allianceRelativeControl(false);

  Command shiftBack = drivebase.driveFieldOriented(shiftBackRobotOriented).withName("Shift Back");

  // Commands to shift robot position at low speed using POV
  SwerveInputStream shiftRightRobotOriented =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(), () -> 0.0, () -> -1 * DriveConstants.POV_SPEED)
          .withControllerRotationAxis(() -> 0.0)
          .robotRelative(true)
          .allianceRelativeControl(false);

  Command shiftRight =
      drivebase.driveFieldOriented(shiftRightRobotOriented).withName("Shift Right");

  // Commands to shift robot position at low speed using POV
  SwerveInputStream shiftLeftRobotOriented =
      SwerveInputStream.of(drivebase.getSwerveDrive(), () -> 0.0, () -> DriveConstants.POV_SPEED)
          .withControllerRotationAxis(() -> 0.0)
          .robotRelative(true)
          .allianceRelativeControl(false);

  Command shiftLeft = drivebase.driveFieldOriented(shiftLeftRobotOriented).withName("Shift Left");

  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Publish subsystem data including commands
    SmartDashboard.putData(drivebase);
    SmartDashboard.putData(robotClaw);
    SmartDashboard.putData(robotElevator);
    SmartDashboard.putData(robotRoller);

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    // Named Commands for Autos
    NamedCommands.registerCommand(
        "LoadCoral",
        robotRoller.loadCoral().withTimeout(2.0).unless(robotRoller::isCoralInsideRoller));
    NamedCommands.registerCommand(
        "ScoreCoral",
        Commands.sequence(
            robotElevator.moveToPosition(ElevatorConstants.ELEVATOR_LEVEL1),
            Commands.race(robotRoller.runReverse().withTimeout(2), robotElevator.holdPosition()),
            robotElevator.moveToPosition(ElevatorConstants.ELEVATOR_LOAD_CORAL),
            Commands.runOnce(robotElevator::disable)));
    NamedCommands.registerCommand(
        "ScoreL2Coral",
        Commands.sequence(
            moveClawAndElevator(
                ClawConstants.CLAW_SAFE_ANGLE_RADS,
                ElevatorConstants.ELEVATOR_LEVEL2,
                ClawConstants.CLAW_LEVEL2_AND_LEVEL3_RADS,
                false),
            Commands.race(robotRoller.runReverse().withTimeout(1.0), robotElevator.holdPosition()),
            moveClawAndElevator(
                ClawConstants.CLAW_SAFE_ANGLE_RADS,
                ElevatorConstants.ELEVATOR_LOAD_CORAL,
                ClawConstants.CLAW_LEVEL1_RADS,
                true),
            Commands.runOnce(robotElevator::disable)));
    NamedCommands.registerCommand("DriveReefLeft", driveToClosestReefLeft);
    NamedCommands.registerCommand("DriveReefRight", driveToClosestReefRight);

    // Setup the auto command chooser using the PathPlanner autos
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // ---------- Driver Controller ----------

    // Change drive type from field oriented to robot oriented, which is similar to tank drive, when
    // 'RB' is pressed on the driver's controller
    driverController.rightBumper().whileTrue(driveRobotOrientedAngularVelocity);

    // Sysid commands
    // driverController.a().whileTrue(drivebase.sysIdAngleMotorCommand());
    // driverController.b().whileTrue(drivebase.sysIdDriveMotorCommand());

    // Drive to the closest position near the reef offset to the left/right when 'A' / 'B' is
    // pressed on the driver's controller
    driverController.a().whileTrue(driveToClosestReefLeft);
    driverController.b().whileTrue(driveToClosestReefRight);

    // lock the wheels in a X pattern while left bumper is held
    driverController
        .leftBumper()
        .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    // Drives the robot slowly to a set position based on which of the pov buttons is pressed on the
    // driver's controller
    driverController.povUp().whileTrue(shiftForward);
    driverController.povDown().whileTrue(shiftBack);
    driverController.povRight().whileTrue(shiftRight);
    driverController.povLeft().whileTrue(shiftLeft);

    // Zero the gyro when 'start' is pressed on the driver's controller
    driverController
        .start()
        .onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance).ignoringDisable(true));

    // ---------- Operator Controller ----------
    // Move the elevator and claw to the level 3 (upper) algae position when the 'POV Up' button is
    // pressed
    // on the operator's controller.
    operatorController
        .povUp()
        .and(() -> !drivebase.isNearReef() || operatorController.getHID().getLeftBumperButton())
        .onTrue(
            moveClawAndElevator(
                    ClawConstants.CLAW_SAFE_ANGLE_RADS,
                    ElevatorConstants.ELEVATOR_LEVEL3_ALGAE,
                    ClawConstants.CLAW_ALGAE_RADS,
                    false)
                .withName("Elevator + Claw: Load Level 3 Algae"));

    // Move the elevator and claw to the level 2 (lower) algae position when the 'POV Right' button
    // is pressed
    // on the operator's controller.
    operatorController
        .povRight()
        .and(() -> !drivebase.isNearReef() || operatorController.getHID().getLeftBumperButton())
        .onTrue(
            moveClawAndElevator(
                    ClawConstants.CLAW_SAFE_ANGLE_RADS,
                    ElevatorConstants.ELEVATOR_LEVEL2_ALGAE,
                    ClawConstants.CLAW_ALGAE_RADS,
                    false)
                .withName("Elevator + Claw: Load Level 2 Algae"));

    // Move the elevator and claw to the load coral position when the 'POV Left' button is pressed
    // on the operator's controller.
    operatorController
        .povLeft()
        .onTrue(
            moveClawAndElevator(
                    ClawConstants.CLAW_SAFE_ANGLE_RADS,
                    ElevatorConstants.ELEVATOR_LOAD_CORAL,
                    ClawConstants.CLAW_LEVEL1_RADS,
                    true)
                .withName("Elevator + Claw: Load Coral"));

    // Move the elevator and claw to the processor position when the 'POV Down' button is pressed
    // on the operator's controller.
    operatorController
        .povDown()
        .and(() -> !drivebase.isNearReef() || operatorController.getHID().getLeftBumperButton())
        .onTrue(
            moveClawAndElevator(
                    ClawConstants.CLAW_PROCESSOR_RADS,
                    ElevatorConstants.ELEVATOR_PROCESSOR,
                    ClawConstants.CLAW_PROCESSOR_RADS,
                    false)
                .withName("Elevator + Claw: Load Processor"));

    // Move the elevator and claw to score in Reef Level 1 when the 'A' button is pressed.
    operatorController
        .a()
        .onTrue(
            moveClawAndElevator(
                    ClawConstants.CLAW_SAFE_ANGLE_RADS,
                    ElevatorConstants.ELEVATOR_LEVEL1,
                    ClawConstants.CLAW_LEVEL1_RADS,
                    false)
                .withName("Elevator + Claw: Move to Coral Level 1"));

    // Move the elevator and claw to score in Reef Level 2 when the 'B' button is pressed.
    operatorController
        .b()
        .onTrue(
            moveClawAndElevator(
                    ClawConstants.CLAW_SAFE_ANGLE_RADS,
                    ElevatorConstants.ELEVATOR_LEVEL2,
                    ClawConstants.CLAW_LEVEL2_AND_LEVEL3_RADS,
                    false)
                .withName("Elevator + Claw: Move to Coral Level 2"));

    // Move the elevator and claw to score in Reef Level 3 when the 'X' button is pressed.
    operatorController
        .x()
        .onTrue(
            moveClawAndElevator(
                    ClawConstants.CLAW_SAFE_ANGLE_RADS,
                    ElevatorConstants.ELEVATOR_LEVEL3,
                    ClawConstants.CLAW_LEVEL2_AND_LEVEL3_RADS,
                    false)
                .withName("Elevator + Claw: Move to Coral Level 3"));

    // Move the elevator and claw to score in Reef Level 4 when the 'Y' button is pressed.
    operatorController
        .y()
        .onTrue(
            moveClawAndElevator(
                    ClawConstants.CLAW_SAFE_ANGLE_RADS,
                    ElevatorConstants.ELEVATOR_LEVEL4,
                    ClawConstants.CLAW_LEVEL4_RADS,
                    false)
                .withName("Elevator + Claw: Move to Coral Level 4"));

    // Run the Roller forward when the right bumper is pressed.
    operatorController
        .rightBumper()
        .whileTrue(robotRoller.runForward().withName("Roller: Run Forward"));

    // Runs the Load Coral Function when Left Trigger is held.
    operatorController.leftTrigger().whileTrue(robotRoller.loadCoral().withName("Loads Coral"));

    // Run the Roller reverse when the right Trigger is pressed.
    operatorController
        .rightTrigger()
        .whileTrue(robotRoller.runReverse().withName("Roller: Run Reverse"));

    // The trigger if we are not in a safe position becomes aborted.
    unsafeTrigger.onTrue(Commands.parallel(robotClaw.abortCommand(), robotElevator.abortCommand()));
  }

  /**
   * Disables all subsystems. This should be called on robot disable to prevent integrator windup in
   * subsystems with PID controllers. It also allows subsystems to setup disabled state so
   * simulation matches RoboRio behavior. Commands are canceled at the Robot level.
   */
  public void disableSubsystems() {
    robotClaw.disable();
    robotElevator.disable();
    robotRoller.disableRoller();
    DataLogManager.log("disableSubsystems");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
    DataLogManager.log("Drive Brake: " + brake);
  }

  /** Get the drive command from the drive subsystem. */
  public Command getTeleopDriveCommand() {
    return Commands.none();
  }

  /** Check if position is safe or unsafe and creates a limit for the robot. */
  public boolean isSafePosition() {
    if (operatorController.getHID().getLeftBumperButton()) {
      return true;
    } else {
      double clawAngle = robotClaw.getAbsoluteAngle();
      double elevatorHeight = robotElevator.getMeasurement();
      if ((clawAngle < 40 && elevatorHeight > Units.inchesToMeters(5))
          || (clawAngle > 60
              && elevatorHeight < Units.inchesToMeters(50)
              && elevatorHeight > Units.inchesToMeters(40))) {
        return false;
      }
      return true;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }

  /**
   * Return a Teleop command with the intention of moving the claw to a safe position, then moving
   * the elevator and the claw to your desired position.
   *
   * @return the command sequence for teleop elevator + claw movements
   */
  public Command moveClawAndElevator(
      double firstClawPos, double elevatorPos, double clawPos, boolean disableElevator) {
    return Commands.sequence(
            Commands.race(robotClaw.moveToPosition(firstClawPos), robotElevator.holdPosition()),
            Commands.race(robotElevator.moveToPosition(elevatorPos), robotClaw.holdPosition()),
            Commands.race(robotClaw.moveToPosition(clawPos), robotElevator.holdPosition()),
            Commands.runOnce(
                () -> {
                  if (disableElevator) {
                    robotElevator.disable();
                  }
                }))
        .onlyIf(safeTrigger);
  }

  /** Set the LEDs to a specified color. */
  public void setLeds(LEDPattern pattern) {
    led.setPattern(pattern);
  }

  /** Set the LEDs to show robot status. */
  public void setLedStatus() {
    if (!isSafePosition()) {
      led.setPattern(LEDPattern.solid(Color.kGray));
    } else if (drivebase.isNearReef()) {
      led.setPattern(LEDPattern.solid(Color.kYellow));
    } else if (robotRoller.isCoralInsideRoller()) {
      led.setPattern(LEDPattern.rainbow(255, 128));
    } else if ((robotElevator.getMeasurement() < Units.inchesToMeters(0.5))
        && (robotClaw.getAbsoluteAngle() < 20.0)) {
      // Safe to load coral
      led.setPattern(LEDPattern.solid(Color.kBlue));
    } else {
      led.setPattern(LEDPattern.solid(Color.kOrange));
    }
  }

  /**
   * Use this to get the PDP for data logging.
   *
   * @return The PowerDistribution module.
   */
  public PowerDistribution getPdp() {
    return this.pdp;
  }

  /**
   * Use this to get the Claw Subsystem.
   *
   * @return a reference to the claw subsystem
   */
  public ClawSubsystem getClawSubsystem() {
    return robotClaw;
  }

  /**
   * Use this to get the Elevator Subsystem.
   *
   * @return a reference to the Elevator Subsystem
   */
  public ElevatorSubsystem getElevatorSubsystem() {
    return robotElevator;
  }

  /**
   * Use this to get the Roller Subsystem.
   *
   * @return a reference to the Roller Subsystem
   */
  public RollerSubsystem getRollerSubsystem() {
    return robotRoller;
  }

  /**
   * Create a command to drive to a position in front of the nearest reef position and shifted by a
   * set amount relative to the face of the reef.
   */
  private Command createDriveReefCommand(Translation2d shift) {
    var nearestPose = drivebase.getPose().nearest(FieldConstants.REEF_POSITIONS);
    var targetPose =
        new Pose2d(
            nearestPose.getTranslation().plus(shift.rotateBy(nearestPose.getRotation())),
            nearestPose.getRotation().minus(new Rotation2d(Math.toRadians(180))));
    DataLogManager.log("Drive to Reef: " + targetPose);
    SmartDashboard.putNumber("Drive to Reef X", targetPose.getX());
    SmartDashboard.putNumber("Drive to Reef Y", targetPose.getY());
    return drivebase.driveToPosePID(targetPose);
  }
}
