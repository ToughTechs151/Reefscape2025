package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The {@code LEDSubsystem} class is a subsystem that controls the color of an LED strip.
 * <pre/>
**/
public class LEDSubsystem extends SubsystemBase {
  private static final int PORT = 9;

  private static final int COLUMNS = 30; // columns

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  /** LED related components for the LEDSubsystem subsystem. */
  public LEDSubsystem() {
    led = new AddressableLED(PORT);
    buffer = new AddressableLEDBuffer(COLUMNS);
    led.setLength(COLUMNS);
    led.start();

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    setDefaultCommand(runPattern(LEDPattern.solid(Color.kGray)).withName("Default"));
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    led.setData(buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(buffer));
  }
}
