package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard. Adapted from FRC Team 3467 Nocturne Code
 */
public class TunableNumber {
  private static final String TABLE_KEY = "TunableNumbers";

  private String key;
  private double defaultValue;
  private boolean hasDefault = false;

  /**
   * Create a new TunableNumber.
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableNumber(String dashboardKey) {
    this.key = TABLE_KEY + "/" + dashboardKey;
  }

  /**
   * Create a new TunableNumber with the default value.
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    setDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void setDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (Constants.TUNING_MODE) {
        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
      }
    }
  }

  /**
   * Get the default value for the number that has been set.
   *
   * @return The default value
   */
  public double getDefault() {
    return defaultValue;
  }

  /**
   * Publishes a new value. Note that the value will not be returned by {@link #get()} until the
   * next cycle.
   *
   * @param value the value to set
   */
  public void set(double value) {
    if (Constants.TUNING_MODE) {
      SmartDashboard.putNumber(key, value);
    } else {
      defaultValue = value;
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    } else {
      return Constants.TUNING_MODE
          ? SmartDashboard.getNumber(key, defaultValue)
          : defaultValue;
    }
  }
}
