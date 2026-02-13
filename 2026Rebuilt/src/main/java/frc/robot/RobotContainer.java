package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.RunFlywheelShooter;
import frc.robot.commands.RunHood;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.Hood;
//import frc.robot.subsystems.Simulation3D;

public class RobotContainer {
  private final FlywheelShooter flywheelShooter = new FlywheelShooter();
  private final Hood hood = new Hood();

  public ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

  private final CommandXboxController operatorController =
      new CommandXboxController(Constants.Xbox.OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();

    flywheelShooter.setDefaultCommand(
        new RunCommand(() -> flywheelShooter.stopMotor(), flywheelShooter)
    );

    hood.configDashboard(matchTab);
  }

  private void configureBindings() {
    //final double kTargetRotorRps = 50.0;
    
    operatorController.a()
    .whileTrue(new RunFlywheelShooter(flywheelShooter, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED1)); //create new speed

    operatorController.x()
    .onTrue(new RunHood(hood, Constants.Hood.HOOD_TEST_SHOT, Constants.Hood.HOOD_TOLERANCE_DEG));
    
    operatorController.y()
    .whileTrue(new RunFlywheelShooter(flywheelShooter, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED2)); //speed change to 50

    operatorController.b()
    .whileTrue(new RunFlywheelShooter(flywheelShooter, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED3)); //speed change to 60

  }

  public Command getAutonomousCommand() {
    return null;
  }
}




