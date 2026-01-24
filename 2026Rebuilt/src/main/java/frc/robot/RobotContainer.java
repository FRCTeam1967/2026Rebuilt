package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.FlywheelShooter;
import frc.robot.Constants;

public class RobotContainer {
  // Subsystems
  private final FlywheelShooter flywheelShooter = new FlywheelShooter();


  public ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

  private final CommandXboxController operatorController =
      new CommandXboxController(Constants.Xbox.OPERATOR_CONTROLLER_PORT);

  /** Creates RobotContainer */
  public RobotContainer() {
    configureBindings();

    flywheelShooter.setDefaultCommand(
        new RunCommand(() -> flywheelShooter.stopMotor(), flywheelShooter)
    );
  }

 
  private void configureBindings() {

    // Pick a target rotor speed (RPS). Start small.
    final double kTargetRotorRps = 50.0; // 50 rps = 3000 rpm (rotor)

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
      double rt = operatorController.getRightTriggerAxis(); // 0..1
      double targetRotorRps = rt * 80.0; // full trigger = 80 rps
      flywheelShooter.setVelocity(targetRotorRps);
    }, flywheelShooter))
    .onFalse(new InstantCommand(() -> flywheelShooter.stopMotor(), flywheelShooter));

  }

  public Command getAutonomousCommand() {
    return null;
  }
}


