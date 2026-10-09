// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autos;
import frc.robot.commands.AlignToHub;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.LocalizationCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
  // One persistent localization command; alignment reads its estimated pose.
  private final LocalizationCommand m_localization = new LocalizationCommand(
      () -> new Rotation2d(),
      RobotContainer::dummyModulePositions);

  // Encoder distances are meters; module order is FL, FR, RL, RR.
  private static SwerveModulePosition[] dummyModulePositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(), new SwerveModulePosition(),
      new SwerveModulePosition(), new SwerveModulePosition()
    };
  }
  // This project has no motor-driving subsystem yet. Replace this requirement with
  // the real drivetrain AND replace previewDriveOutput with its robot-relative drive method.
  private final SubsystemBase m_alignmentPreview = new SubsystemBase("Alignment Preview") {};

  private void previewDriveOutput(ChassisSpeeds speeds) {
    // Preview only: no motors are driven and no simulated movement is fed into localization.
    SmartDashboard.putNumber("Robot/HubAlignment/Requested Omega rad per sec",
        speeds.omegaRadiansPerSecond);
  }

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putBoolean("Robot/Localization/UsingDummyOdometry", true);
    SmartDashboard.putBoolean("Robot/HubAlignment/Preview Only", true);
    configureBindings();
    CommandScheduler.getInstance().schedule(m_localization);
    // Restore the continuous command after cancelAll(), including the existing testInit().
    CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
      if (!m_localization.isScheduled()) {
        CommandScheduler.getInstance().schedule(m_localization);
      }
    });
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Hold A to face the alliance hub; releasing A stops the rotation request.
    m_driverController.a().whileTrue(new AlignToHub(
        m_alignmentPreview,
        m_localization::getRobotPose,
        m_localization::hasValidPoseSensorResult,
        this::previewDriveOutput));

    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
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
