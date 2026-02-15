package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Pivot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;


//import frc.robot.subsystems.Simulation3D;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Pivot pivot = new Pivot();
  public final Intake intake = new Intake();
  public final Indexer indexer = new Indexer();
  public final Feeder feeder = new Feeder();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  //public final Joystick joystick = new Joystick(0);

  public ShuffleboardTab fieldTab = Shuffleboard.getTab("Field");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final FlywheelShooter flywheelShooter = new FlywheelShooter();
  private final Hood hood = new Hood();

  public ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

  public RobotContainer() {
    configureBindings();

    flywheelShooter.setDefaultCommand(
        new RunCommand(() -> flywheelShooter.stopMotor(), flywheelShooter)
    );

    hood.configDashboard(matchTab);
    pivot.configDashboard(fieldTab);
  }

  private void configureBindings() {
    //final double kTargetRotorRps = 50.0;
    
    //SHOOTER AND HOOD BUTTON BINDINGS
    m_operatorController.x()
    .whileTrue(
      new ParallelCommandGroup(
        // new ParallelCommandGroup(
        //   new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED),
        //   new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED)
        // ),

        new RunFlywheelShooter(flywheelShooter, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
        
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> flywheelShooter.reachedShooterSpeed()),
          new ParallelCommandGroup(
            new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED),
            new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED)) 
        )
      )
    );      

    m_operatorController.y()
    .onTrue(new RunHood(hood, Constants.Hood.HOOD_MAX*Constants.Hood.PERCENT_UP, Constants.Hood.HOOD_TOLERANCE_DEG));
    
    m_operatorController.rightBumper()
    .onTrue(new RunHood(hood, 0, Constants.Hood.HOOD_TOLERANCE_DEG));
    
    //INTAKE AND INDEXER BUTTON BINDINGS
    m_operatorController.leftTrigger().whileTrue(new RunIntake(intake, Constants.Intake.INTAKE_MOTOR_SPEED));

    m_operatorController.rightTrigger().whileTrue(
      new ParallelCommandGroup(
        new RunIntake(intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
        new RunIndexer(indexer, 10.0),
        new RunFeeder(feeder, -Constants.Feeder.FEEDER_SPEED)));

    m_operatorController.b().onTrue(new MovePivot(pivot, Constants.Pivot.DOWN_POSITION));

    m_operatorController.a().onTrue(new MovePivot(pivot, Constants.Pivot.SAFE));

    //SIMULATION BUTTON BINDINGS
    //new JoystickButton(joystick, 1).onTrue(new MovePivot(pivot, 8));
    //new JoystickButton(joystick, 2).onTrue(new MovePivot(pivot, 0));

  }

  //  public void maintainHood() { 
  //    hood.setRelToAbs();
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}




