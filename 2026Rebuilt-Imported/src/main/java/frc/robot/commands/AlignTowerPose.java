// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.command2.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.Alliance;
import frc.robot.generated.TunerConstants;
import dev.doglog.DogLog;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignTowerPose extends Command {
  private final SwerveOnTheseBows swerve;

  private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
      
  final StructPublisher<Pose2d> towerPublisher = NetworkTableInstance.getDefault().getTable("alignment").getStructTopic("tower", Pose2d.struct).publish();  
  
  private static final double kP_translational = 2.5; //0.85
  private static final double kP_rotational = 0.85;
  private Transform2d difference = new Transform2d();

  public AlignTowerPose(SwerveOnTheseBows swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d drivetrainPose = swerve.getPose();
    // 0.84, 4.8, 0

    difference = VisabelleUpdate.towerPose.minus(drivetrainPose);
    // 1.092, 4.61, 3.14
    if (DriverStation.getAlliance().get() == Alliance.Red) {
        // DogLog.log("Pose difference: ", difference);
        // DogLog.log("pose diff x", difference.getX());
        // DogLog.log("pose diff y", difference.getY());

        double xVelocity = MathUtil.clamp(-difference.getX() * kP_translational, -MaxSpeed, MaxSpeed);
        //DogLog.log("xVelocity: ", xVelocity);

        double yVelocity = MathUtil.clamp(-difference.getY() * kP_translational, -MaxSpeed, MaxSpeed);
        //DogLog.log("yVelocity: ", yVelocity);

        double rotationalVelocity = MathUtil.clamp(difference.getRotation().getRadians() * kP_rotational, -MaxAngularRate, MaxAngularRate);
        //DogLog.log("rotationalVelocity: ", rotationalVelocity);

        ChassisVelocities alignmentSpeed = ChassisVelocities.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, drivetrainPose.getRotation());
        //DogLog.log("alignmentSpeed: ", alignmentSpeed);

        swerve.setControl(request.withSpeeds(alignmentSpeed));
        // DogLog.log("x isFinished: ", Math.abs(difference.getX()) < 0.05);
        // DogLog.log("y isFinished: ", Math.abs(difference.getY()) < 0.05);
        // DogLog.log("rotation isFinished: ", Math.abs(difference.getRotation().getRadians()) < Units.degreesToRadians(2));
    }
    else {
        // DogLog.log("Pose difference: ", difference);
        // DogLog.log("pose diff x", difference.getX());
        // DogLog.log("pose diff y", difference.getY());

        double xVelocity = MathUtil.clamp(difference.getX() * kP_translational, -MaxSpeed, MaxSpeed);
        // DogLog.log("xVelocity: ", xVelocity);

        double yVelocity = MathUtil.clamp(difference.getY() * kP_translational, -MaxSpeed, MaxSpeed);
        // DogLog.log("yVelocity: ", yVelocity);

        double rotationalVelocity = MathUtil.clamp(difference.getRotation().getRadians() * kP_rotational, -MaxAngularRate, MaxAngularRate);
        // DogLog.log("rotationalVelocity: ", rotationalVelocity);

        ChassisVelocities alignmentSpeed = ChassisVelocities.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, drivetrainPose.getRotation());
        // DogLog.log("alignmentSpeed: ", alignmentSpeed);

        swerve.setControl(request.withSpeeds(alignmentSpeed));
        // DogLog.log("x isFinished: ", Math.abs(difference.getX()) < 0.05);
        // DogLog.log("y isFinished: ", Math.abs(difference.getY()) < 0.05);
        // DogLog.log("rotation isFinished: ", Math.abs(difference.getRotation().getRadians()) < Units.degreesToRadians(2));
    }    
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