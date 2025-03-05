/* The {@code LEDSubsystem} class is a subsystem that controls the color of an LED strip.
 * <p>
 * This subsystem utilizes WPILib's {@link AddressableLED}
 *  and {@link AddressableLEDBuffer} to manage an LED strip.
 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The {@code LEDSubsystem} class defines components and methods to manage an LED strip connected to
 * the robot.
 */
public class LEDSubsystem extends SubsystemBase {
  /** The port number to which the LED strip is connected. */
  private static final int PORT = 0;

  /** The number of LED columns in the strip. */
  private static final int COLUMNS = 42;

  /** The {@link AddressableLED} object used to control the LED strip. */
  private final AddressableLED led;

  /** The {@link AddressableLEDBuffer} object to manage LED color data. */
  private final AddressableLEDBuffer buffer;

  private LEDPattern pattern;

  /**
   * Constructs the LEDSubsystem and initializes the LED strip and buffer. Sets the default command
   * to turn the strip off or display a default color.
   */
  public LEDSubsystem() {
    led = new AddressableLED(PORT);
    buffer = new AddressableLEDBuffer(COLUMNS);
    led.setLength(COLUMNS);
    led.start();
    setPattern(LEDPattern.solid(Color.kBlack));
  }

  /** Periodically sends the latest LED color data to the LED strip for display. */
  @Override
  public void periodic() {
    pattern.applyTo(buffer);
    led.setData(buffer);
  }

  /**
   * Creates a command to run a specific LED pattern on the strip.
   *
   * @param pattern the {@link LEDPattern} to apply to the LED strip
   * @return a {@link Command} that sets the pattern for the LED strip
   */
  public Command runPattern(LEDPattern pattern) {
    return runOnce(() -> this.pattern = pattern);
  }

  /**
   * Set a specific LED pattern on the strip.
   *
   * @param pattern the {@link LEDPattern} to apply to the LED strip
   */
  public void setPattern(LEDPattern pattern) {
    this.pattern = pattern;
  }
}
