package frc.robot.subsystems;



import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private static final int kPort = 0; //TODO: update port
  private static final int kLength = 10; //TODO: update length

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  public LED() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    m_led.setData(m_buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   * @param pattern the LED pattern to run
   * @return the command created
   */
  public Command runPattern(LEDPattern pattern) {
    Command runCommand = run(() -> pattern.applyTo(m_buffer));
    runCommand.addRequirements(this);
    return runCommand;
  }

  /**
   * applies pattern to strip
   * @param pattern
   */
  public void applyPattern(LEDPattern pattern) {
    pattern.applyTo(m_buffer);
  }
}