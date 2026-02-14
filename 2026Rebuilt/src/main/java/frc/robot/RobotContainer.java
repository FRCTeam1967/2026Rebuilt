package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.RunFlywheelShooter;
import frc.robot.commands.RunHood;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.Hood;
//import frc.robot.subsystems.Simulation3D;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  Pivot pivot = new Pivot();
  Intake intake = new Intake();
  Indexer indexer = new Indexer();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  public final Joystick joystick = new Joystick(0);

  public ShuffleboardTab fieldTab = Shuffleboard.getTab("Field");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
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

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().onTrue(m_exampleSubsystem.exampleMethodCommand());
    m_operatorController.x().whileTrue(new RunIntake(intake, Constants.Intake.INTAKE_MOTOR_SPEED));
    m_operatorController.rightTrigger().whileTrue(new ParallelCommandGroup(new RunIntake(intake, Constants.Intake.INTAKE_MOTOR_SPEED), new RunIndexer(indexer, 10.0)));
    m_operatorController.b().onTrue(new MovePivot(pivot, Constants.Pivot.DOWN_POSITION));
    m_operatorController.a().onTrue(new MovePivot(pivot, Constants.Pivot.SAFE));

    //SIMULATION BUTTON BINDINGS
    //new JoystickButton(joystick, 1).onTrue(new MovePivot(pivot, 8));
    //new JoystickButton(joystick, 2).onTrue(new MovePivot(pivot, 0));

    pivot.configDashboard(fieldTab);
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}




