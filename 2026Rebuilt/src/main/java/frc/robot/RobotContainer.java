// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static final CANBus CANBus = new CANBus("Default Name");

  private final CommandXboxController operatorController = new CommandXboxController(1);

  public final Shooter shooter = new Shooter();
  public final Feeder feeder = new Feeder();
  public final Indexer indexer = new Indexer();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings.
   */

  private void configureBindings() {
    //SHOOTER
    operatorController.______________().whileTrue( // choose a button to bind shooting to!
        new ParallelCommandGroup(
            new _____________(shooter, Constants.Shooter.____________), // run the shooter at the shooter speed defined in constants!
            new SequentialCommandGroup(
                new RunFeeder(feeder, Constants.Feeder.PREP_FEEDER).withTimeout(0.5),
                                
                new ParallelCommandGroup(
                    new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED),
                    new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED)
                )
            )
        )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}