package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.LED;

public class RobotContainer {
  private final LED leds = new LED();
  private final CommandXboxController driver = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driver.a().onTrue(new InstantCommand(leds::setRainbow, leds));
    driver.b().onTrue(new InstantCommand(leds::setFire, leds));
    driver.x().onTrue(new InstantCommand(leds::setTwinkle, leds));
    driver.leftBumper().onTrue(new InstantCommand(leds::setColorFlow, leds));
    driver.rightBumper().onTrue(new InstantCommand(leds::setLarson, leds));
  }
}
