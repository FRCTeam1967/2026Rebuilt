package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.RunFlywheelShooter;
import frc.robot.subsystems.FlywheelShooter;

public class RobotContainer {

  // Controller
  private final XboxController operatorController =
      new XboxController(0);

  // Subsystem
  private final FlywheelShooter shooter = new FlywheelShooter();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
//
  }
}

