// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import dev.doglog.DogLog;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignTowerPose extends Command {
  private final SwerveOnTheseBows swerve;

  private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
      
  Optional<Alliance> ally = DriverStation.getAlliance();
  final StructPublisher<Pose2d> towerPublisher = NetworkTableInstance.getDefault().getTable("alignment").getStructTopic("tower", Pose2d.struct).publish();  
  
  //tower pose - RED ALLIANCE

  private static final double kP_translational = 0.85;
  private static final double kP_rotational = 0.85;
  private Transform2d difference = new Transform2d();

  public AlignTowerPose(SwerveOnTheseBows swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (ally.isPresent()) {
        // if (DriverStation.getAlliance().get() == Alliance.Red) {
        //     towerPose = new Pose2d(15.421048, 3.432656, new Rotation2d(0));
        //     //DogLog.log("Tower Pose: ", towerPose);
        // }
        // if (DriverStation.getAlliance().get() == Alliance.Blue) {
        //     towerPose = new Pose2d(1.092, 4.61, new Rotation2d(Math.PI));
        //     //DogLog.log("Tower Pose: ", towerPose);
        // }
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d drivetrainPose = swerve.getPose();
    // 0.84, 4.8, 0

    difference = VisabelleUpdate.towerpose.minus(drivetrainPose);
    // 1.092, 4.61, 3.14
        DogLog.log("Pose difference: ", difference);
        DogLog.log("pose diff x", difference.getX());
        DogLog.log("pose diff y", difference.getY());

        double xVelocity = MathUtil.clamp(-difference.getX() * kP_translational, -MaxSpeed, MaxSpeed);
        DogLog.log("xVelocity: ", xVelocity);

        double yVelocity = MathUtil.clamp(-difference.getY() * kP_translational, -MaxSpeed, MaxSpeed);
        DogLog.log("yVelocity: ", yVelocity);

        double rotationalVelocity = MathUtil.clamp(difference.getRotation().getRadians() * kP_rotational, -MaxAngularRate, MaxAngularRate);
        DogLog.log("rotationalVelocity: ", rotationalVelocity);

        ChassisSpeeds alignmentSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, drivetrainPose.getRotation());
        DogLog.log("alignmentSpeed: ", alignmentSpeed);

        swerve.setControl(request.withSpeeds(alignmentSpeed));
        DogLog.log("x isFinished: ", Math.abs(difference.getX()) < 0.05);
        DogLog.log("y isFinished: ", Math.abs(difference.getY()) < 0.05);
        DogLog.log("rotation isFinished: ", Math.abs(difference.getRotation().getRadians()) < Units.degreesToRadians(2));

    // if (ally.get() == Alliance.Red) {
    //     DogLog.log("Pose difference: ", difference);
    //     DogLog.log("pose diff x", difference.getX());
    //     DogLog.log("pose diff y", difference.getY());

    //     double xVelocity = MathUtil.clamp(difference.getX() * kP_translational, -MaxSpeed, MaxSpeed);
    //     DogLog.log("xVelocity: ", xVelocity);

    //     double yVelocity = MathUtil.clamp(difference.getY() * kP_translational, -MaxSpeed, MaxSpeed);
    //     DogLog.log("yVelocity: ", yVelocity);

    //     double rotationalVelocity = MathUtil.clamp(difference.getRotation().getRadians() * kP_rotational, -MaxAngularRate, MaxAngularRate);
    //     DogLog.log("rotationalVelocity: ", rotationalVelocity);

    //     ChassisSpeeds alignmentSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, drivetrainPose.getRotation());
    //     DogLog.log("alignmentSpeed: ", alignmentSpeed);

    //     drivetrain.setControl(request.withSpeeds(alignmentSpeed));
    //     DogLog.log("x isFinished: ", Math.abs(difference.getX()) < 0.05);
    //     DogLog.log("y isFinished: ", Math.abs(difference.getY()) < 0.05);
    //     DogLog.log("rotation isFinished: ", Math.abs(difference.getRotation().getRadians()) < Units.degreesToRadians(2));
    // }
    // else {
    //       // Pose2d drivetrainPose = drivetrain.getPose();
    //       // // 0.84, 4.8, 0

    //       // difference = towerPose.minus(drivetrainPose);
    //       // // 0.252, 0.19, -3.14

    //     DogLog.log("Pose difference: ", difference); // // 0.252, 0.19, -3.14
    //     DogLog.log("pose diff x", difference.getX());
    //     DogLog.log("pose diff y", difference.getY());

    //     double xVelocity = MathUtil.clamp(-difference.getX() * kP_translational, -MaxSpeed, MaxSpeed); //-0.189
    //     DogLog.log("xVelocity: ", xVelocity);

    //     double yVelocity = MathUtil.clamp(-difference.getY() * kP_translational, -MaxSpeed, MaxSpeed); //-0.1425
    //     DogLog.log("yVelocity: ", yVelocity);

    //     double rotationalVelocity = MathUtil.clamp(difference.getRotation().getRadians() * kP_rotational, -MaxAngularRate, MaxAngularRate); // -2.355
    //     DogLog.log("rotationalVelocity: ", rotationalVelocity);

    //     ChassisSpeeds alignmentSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, drivetrainPose.getRotation());
    //     DogLog.log("alignmentSpeed: ", alignmentSpeed);

    //     drivetrain.setControl(request.withSpeeds(alignmentSpeed));
    //     DogLog.log("x isFinished: ", Math.abs(difference.getX()) < 0.05);
    //     DogLog.log("y isFinished: ", Math.abs(difference.getY()) < 0.05);
    //     DogLog.log("rotation isFinished: ", Math.abs(difference.getRotation().getRadians()) < Units.degreesToRadians(2));

    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      swerve.setControl(request.withSpeeds(new ChassisSpeeds()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return Math.abs(difference.getX()) < 0.05 &&
             Math.abs(difference.getY()) < 0.05 &&
             Math.abs(difference.getRotation().getRadians()) < Units.degreesToRadians(2);
  }
}