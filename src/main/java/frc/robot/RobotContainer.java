// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

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

  private final LEDSubsystem led = new LEDSubsystem();

  private final ArmSubsystem robotArm = new ArmSubsystem(ArmSubsystem.initializeHardware());

  private final IntakeSubsystem robotIntake =
      new IntakeSubsystem(IntakeSubsystem.initializeHardware());

  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Publish subsystem data including commands
    SmartDashboard.putData(robotArm);
    SmartDashboard.putData(robotIntake);

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

    // ---------- Operator Controller ----------
    // Move the arm to the low position when the 'A' button is pressed on the operator's controller.
    operatorController
        .a()
        .onTrue(
            robotArm
                .moveToPosition(Constants.ArmConstants.ARM_FORWARD_POSITION_RADS)
                .andThen(robotArm::disable)
                .withName("Arm: Move to Forward Position"));

    // Move the arm to the high position when the 'B' button is pressed on the operator's
    // controller.
    operatorController
        .b()
        .onTrue(
            robotArm
                .moveToPosition(Constants.ArmConstants.ARM_BACK_POSITION_RADS)
                .andThen(robotArm::disable)
                .withName("Arm: Move to Back Position"));

    // Disable the arm controller when the 'X' button is pressed on the operator's controller.
    // NOTE: This is intended for initial arm testing and should be removed in the final robot
    // to prevent accidental disable resulting in lowering of the arm.
    operatorController.x().onTrue(Commands.runOnce(robotArm::disable));

    // Run the intake forward when the right bumper is pressed.
    operatorController
        .rightBumper()
        .whileTrue(robotIntake.runForward().withName("Intake: Run Forward"));

    // Run the intake in reverse when the left bumper is pressed.
    operatorController
        .leftBumper()
        .whileTrue(robotIntake.runReverse().withName("Intake: Run Reverse"));

    // Run the intake until a note is loaded and move the arm back when the Y button is pressed and
    // held.
    operatorController
        .y()
        .whileTrue(
            robotIntake
                .loadNote()
                .andThen(robotArm.moveToPosition(Constants.ArmConstants.ARM_BACK_POSITION_RADS))
                .andThen(robotArm::disable)
                .withName("Intake: Load Note"));
  }

  /**
   * Disables all subsystems. This should be called on robot disable to prevent integrator windup in
   * subsystems with PID controllers. It also allows subsystems to setup disabled state so
   * simulation matches RoboRio behavior. Commands are canceled at the Robot level.
   */
  public void disableSubsystems() {
    robotArm.disable();
    robotIntake.disableIntake();
    DataLogManager.log("disableSubsystems");
  }

  /** Get the drive command from the drive subsystem. */
  public Command getTeleopDriveCommand() {
    return Commands.none();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return new PrintCommand("No Auto Selected");
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
   * Use this to get the Arm Subsystem.
   *
   * @return a reference to the arm subsystem
   */
  public ArmSubsystem getArmSubsystem() {
    return robotArm;
  }

  /**
   * Use this to get the Intake Subsystem.
   *
   * @return a reference to the Intake Subsystem
   */
  public IntakeSubsystem getIntakeSubsystem() {
    return robotIntake;
  }
}
