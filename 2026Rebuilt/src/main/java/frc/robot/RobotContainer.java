package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {
  private final LEDSubsystem leds = new LEDSubsystem();
  private final CommandXboxController driver = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driver.a().onTrue(new InstantCommand(leds::setRainbow, leds));
    driver.b().onTrue(new InstantCommand(leds::setFire, leds));
  }
}
