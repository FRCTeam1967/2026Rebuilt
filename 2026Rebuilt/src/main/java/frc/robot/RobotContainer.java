// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autos;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.ExampleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static final LinearVelocityUnit MetersPerSecond = null;
  public final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
       public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
       public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity  
       /* Setting up bindings for necessary control of the swerve drive platform */
       public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
           .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
           .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
      
       public final SwerveRequest.FieldCentricFacingAngle driveAtAngle = new SwerveRequest.FieldCentricFacingAngle()
           .withDeadband(MaxSpeed * 0.1) // 0.1 = deadband
           .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
      
       private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
       private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
               // Drivetrain will execute this command periodically
               swerve.applyRequest(() ->
                   drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                       .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                       .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
               )
           );


           //BUTTON BINDING: REZERO
           m_driverController.start().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));//.seedFieldCentric()


           //BUTTON BINDING: POV buttons
           m_driverController.povUp().whileTrue(swerve.applyRequest(() ->
           drive.withVelocityX(0.2 * MaxSpeed) // Drive forward with negative Y (forward)
               .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
               .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
           ));


           m_driverController.povDown().whileTrue(swerve.applyRequest(() ->
           drive.withVelocityX(-0.2 * MaxSpeed) // Drive forward with negative Y (forward)
               .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
               .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
           ));


           m_driverController.povRight().whileTrue(swerve.applyRequest(() ->
           drive.withVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
               .withVelocityY(-0.2 * MaxSpeed) // Drive left with negative X (left)
               .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
           ));


           m_driverController.povLeft().whileTrue(swerve.applyRequest(() ->
           drive.withVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
               .withVelocityY(0.2 * MaxSpeed) // Drive left with negative X (left)
               .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
           ));


           // Idle while the robot is disabled. This ensures the configured
           // neutral mode is applied to the drive motors while disabled.
           final var idle = new SwerveRequest.Idle();
           RobotModeTriggers.disabled().whileTrue(
               swerve.applyRequest(() -> idle).ignoringDisable(true)
           );

           //BUTTON BINDING: DEFENSE MODE
           m_driverController.leftBumper().whileTrue(swerve.applyRequest(() -> brake));
          
           // m_driverController.x().whileTrue(
           //     new ConditionalCommand(new RunCommand(() -> gerryRig.runMotor(0.7), gerryRig),
           //         new RunCommand(() -> gerryRig.stopMotor(), gerryRig),
           //         () -> autoes.getDisSensor() <= 0.15)
           // );


           // Run SysId routines when holding back/start and X/Y.
           // Note that each routine should be run exactly once in a single log.
           m_driverController.povDown().and(m_driverController.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
           m_driverController.povDown().and(m_driverController.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
           m_driverController.povUp().and(m_driverController.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
           m_driverController.povUp().and(m_driverController.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));


    // Hold A to face the alliance hub; releasing A stops the rotation request.
  }
}
