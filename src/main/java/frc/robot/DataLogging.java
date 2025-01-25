package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Map;

/** The DataLogging class contains all the logic for using telemetry. */
public class DataLogging {

  private DoubleLogEntry loopTime;
  private double startTime;
  private ShuffleboardLayout pdpWidget;
  private boolean everBrownout = false;
  private boolean prevDsConnectState;

  private DataLogging() {
    // Starts recording to data log
    DataLogManager.start();
    final DataLog log = DataLogManager.getLog();

    // Record both DS control and joystick data. To
    DriverStation.startDataLog(DataLogManager.getLog(), Constants.LOG_JOYSTICK_DATA);

    ShuffleboardTab sbRobotTab = Shuffleboard.getTab("Robot");
    pdpWidget = sbRobotTab.getLayout("PDP", BuiltInLayouts.kGrid).withSize(3, 4).withPosition(3, 0);
    ShuffleboardLayout rcWidget =
        sbRobotTab.getLayout("RobotController", BuiltInLayouts.kGrid).withSize(3, 3);

    /* sbRobotTab */
    rcWidget
        .addNumber("Batt Volt", RobotController::getBatteryVoltage)
        .withWidget(BuiltInWidgets.kVoltageView)
        .withProperties(Map.of("min", 0, "max", 13));
    rcWidget
        .addBoolean("Brown Out", RobotController::isBrownedOut)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "Red", "Color when false", "Green"));
    rcWidget
        .addBoolean("Ever Browned Out", this::getEverBrownOut)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "Red", "Color when false", "Green"));

    prevDsConnectState = DriverStation.isDSAttached();

    DataLogManager.log(String.format("Brownout Voltage: %f", RobotController.getBrownoutVoltage()));

    // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish

    StringLogEntry commandLog = new StringLogEntry(log, "/command/event");
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> commandLog.append("Command initialized:" + command.getName()));

    if (Constants.COMMAND_EXECUTE_LOG) {
      CommandScheduler.getInstance()
          .onCommandExecute(command -> commandLog.append("Command execute:" + command.getName()));
    }

    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> commandLog.append("Command interrupted:" + command.getName()));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> commandLog.append("Command finished:" + command.getName()));
    commandLog.append("Opened command log");

    loopTime = new DoubleLogEntry(log, "/robot/LoopTime");
  }

  private static class InstanceHolder {
    private static final DataLogging instance = new DataLogging();
  }

  /**
   * Gets the datalogging Singleton object.
   *
   * @return DataLogging
   */
  public static DataLogging getInstance() {
    return InstanceHolder.instance;
  }

  /**
   * Runs at each loop slice.. This method should be called in the robotPeriodic method in
   * Robot.java. the code must be the last thing in the method.
   *
   * <pre>{@code
   * //must be at end
   * datalog.periodic();
   * }</pre>
   */
  public void periodic() {

    if (RobotController.isBrownedOut()) {
      everBrownout = true;
    }

    boolean newDsConnectState = DriverStation.isDSAttached();
    if (prevDsConnectState != newDsConnectState) {
      Shuffleboard.addEventMarker(
          "Driver Station is %s" + (newDsConnectState ? "Connected" : "Disconnected"),
          EventImportance.kHigh);
      prevDsConnectState = newDsConnectState;
    }

    if (Constants.LOOP_TIMING_LOG) {
      loopTime.append(Timer.getFPGATimestamp() - startTime);
    }
  }

  /**
   * Called from robot.java immediately after the robotContainer is created.
   *
   * @param robotContainer The robotContainer just constructed.
   */
  public void dataLogRobotContainerInit(RobotContainer robotContainer) {

    // Add hardware sendables here
    PowerDistribution pdp = robotContainer.getPdp();
    pdpWidget.add("PDP", pdp).withWidget(BuiltInWidgets.kPowerDistribution);

    // Log configuration info here
    DataLogManager.log(String.format("PDP Can ID: %d", pdp.getModule()));

    // Add values with supplier functions here.
    pdpWidget
        .addNumber("PDP Temp", pdp::getTemperature)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 15, "max", 50));
  }

  public void startLoopTime() {
    startTime = Timer.getFPGATimestamp();
  }

  public final boolean getEverBrownOut() {
    return this.everBrownout;
  }
}
