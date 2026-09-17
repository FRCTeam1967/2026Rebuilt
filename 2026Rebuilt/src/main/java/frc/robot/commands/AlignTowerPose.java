// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**Creates a new AlignTowerPose */
public class AlignTowerPose extends Command {
  private final SwerveOnTheseBows swerve;

  private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
      
  final StructPublisher<Pose2d> towerPublisher = NetworkTableInstance.getDefault().getTable("alignment").getStructTopic("tower", Pose2d.struct).publish();  
  
  private static final double kP_translational = 2.5; //0.85
  private static final double kP_rotational = 0.85;
  private Transform2d difference = new Transform2d();

  /** 
   * @param Swerve subsystem
  */
  public AlignTowerPose(SwerveOnTheseBows swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  /**Called when the command is initially scheduled*/
  @Override
  public void initialize() {
  }

  /**Called every time the scheduler runs while the command is scheduled.*/
  @Override
  public void execute() {
    Pose2d drivetrainPose = swerve.getPose();

    difference = VisabelleUpdate.towerPose.minus(drivetrainPose);

    if (DriverStation.getAlliance().get() == Alliance.Red) {

        double xVelocity = MathUtil.clamp(-difference.getX() * kP_translational, -MaxSpeed, MaxSpeed);
        double yVelocity = MathUtil.clamp(-difference.getY() * kP_translational, -MaxSpeed, MaxSpeed);
        double rotationalVelocity = MathUtil.clamp(difference.getRotation().getRadians() * kP_rotational, -MaxAngularRate, MaxAngularRate);

        ChassisSpeeds alignmentSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, drivetrainPose.getRotation());

        swerve.setControl(request.withSpeeds(alignmentSpeed));
    }
    else {
        double xVelocity = MathUtil.clamp(difference.getX() * kP_translational, -MaxSpeed, MaxSpeed);
        double yVelocity = MathUtil.clamp(difference.getY() * kP_translational, -MaxSpeed, MaxSpeed);
        double rotationalVelocity = MathUtil.clamp(difference.getRotation().getRadians() * kP_rotational, -MaxAngularRate, MaxAngularRate);

        ChassisSpeeds alignmentSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, drivetrainPose.getRotation());

        swerve.setControl(request.withSpeeds(alignmentSpeed));
    }    
  }

  /**Called once the command ends or is interrupted.*/
  @Override
  public void end(boolean interrupted) {
      swerve.setControl(request.withSpeeds(new ChassisSpeeds()));
  }

  /**
   * @return true if the absolute X and Y errors are less than 0.05 and absolute rotational error is less than 2 degrees
  */
  @Override
  public boolean isFinished() {
      return Math.abs(difference.getX()) < 0.05 &&
             Math.abs(difference.getY()) < 0.05 &&
             Math.abs(difference.getRotation().getRadians()) < Units.degreesToRadians(2);
  }
}