// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import dev.doglog.DogLog;
//import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignTowerPose extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final VisionUpdate vision;
  private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();
  private final RobotContainer m_robotContainer = new RobotContainer();
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final AutoFactory autoFactory = new AutoFactory(
      m_robotContainer.drivetrain::getPose, // A function that returns the current robot pose
      m_robotContainer.drivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
      m_robotContainer.drivetrain::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
      m_robotContainer.drivetrain // The drive subsystem
  );

  AutoRoutine routine = autoFactory.newRoutine("TowerDepot"); // TODO: change name depending on side?
  AutoTrajectory tower_path = routine.trajectory("TowerDepot");
  
  public AlignTowerPose(CommandSwerveDrivetrain drivetrain, VisionUpdate vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d towerPose = tower_path.getFinalPose().get();
    Pose2d drivetrainPose = m_robotContainer.drivetrain.getPose();

    Transform2d difference = towerPose.minus(drivetrainPose);

    DogLog.log("Pose difference: ", difference);

    drivetrain.applyRequest(() ->
      drive.withVelocityX(difference.getX() * MaxSpeed) // Drive forward with negative Y (forward)
         .withVelocityY(difference.getY() * MaxSpeed) // Drive left with negative X (left)
         .withRotationalRate(difference.getRotation().getDegrees() * MaxAngularRate)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
