package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.Simulation3D;

public class RobotContainer {
  private final FlywheelShooter flywheelShooter = new FlywheelShooter();
  private final Simulation3D simulation = new Simulation3D(); // ✅ ensures logging code exists

  public ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

  private final CommandXboxController operatorController =
      new CommandXboxController(Constants.Xbox.OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();

    flywheelShooter.setDefaultCommand(
        new RunCommand(() -> flywheelShooter.stopMotor(), flywheelShooter)
    );
  }

  private void configureBindings() {
    final double kTargetRotorRps = 50.0;

    operatorController.leftTrigger()
        .whileTrue(new RunCommand(() -> flywheelShooter.setVelocity(kTargetRotorRps), flywheelShooter))
        .onFalse(new InstantCommand(() -> flywheelShooter.stopMotor(), flywheelShooter));

    operatorController.rightTrigger(0.2)
        .whileTrue(new RunCommand(() -> flywheelShooter.setMotor(0.40), flywheelShooter))
        .onFalse(new InstantCommand(() -> flywheelShooter.stopMotor(), flywheelShooter));

    operatorController.a()
        .whileTrue(new RunCommand(() -> flywheelShooter.setMotor(0.70), flywheelShooter))
        .onFalse(new InstantCommand(() -> flywheelShooter.stopMotor(), flywheelShooter));

    operatorController.b()
        .whileTrue(new RunCommand(() -> flywheelShooter.setMotor(0.20), flywheelShooter))
        .onFalse(new InstantCommand(() -> flywheelShooter.stopMotor(), flywheelShooter));

    operatorController.x()
        .onTrue(new InstantCommand(() -> flywheelShooter.stopMotor(), flywheelShooter));

    operatorController.rightTrigger(0.05)
        .whileTrue(new RunCommand(() -> {
          double rt = operatorController.getRightTriggerAxis();
          double targetRotorRps = rt * 80.0;
          flywheelShooter.setVelocity(targetRotorRps);
        }, flywheelShooter))
        .onFalse(new InstantCommand(() -> flywheelShooter.stopMotor(), flywheelShooter));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}




