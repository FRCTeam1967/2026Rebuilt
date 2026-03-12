// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;  
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

public class Visabelle extends SubsystemBase {
  private double maxAngularRate;
  private SwerveOnTheseBows swerve;

  private DoublePublisher visionDist;
  //private DoubleSupplier visionDist;
  private DoublePublisher visionBlueDist;
  private DoublePublisher visionRedDist;
  private BooleanPublisher allianceIsBlue;

  private Translation2d hubPose;
  // = new Translation2d(11.914324760437012, 4.033950328826904);

  public Visabelle(SwerveOnTheseBows drivetrain, double maxAngularRate) {
    this.swerve = drivetrain;
    this.maxAngularRate = maxAngularRate;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("vision distances");
    visionDist = table.getDoubleTopic("dist to hub").publish();
    visionBlueDist = table.getDoubleTopic("dist to blue hub").publish();
    visionRedDist = table.getDoubleTopic("dist to red hub").publish();
    allianceIsBlue = table.getBooleanTopic("allianceIsBlue").publish();
  }

  public double limelight_aim_proportional() {        
      double kP = 0.02; //0.035
      double targetingAngularVelocity = 0.0; 
      // tx ranges from (-hfov/2) to (hfov/2) in degrees.

      targetingAngularVelocity = (LimelightHelpers.getTX("limelight-front") * kP);

      // convert to radians per second for our drive method
      targetingAngularVelocity *= maxAngularRate;
      //invert since tx is positive when the target is to the right of the crosshair
      targetingAngularVelocity *= -1.0;
      return targetingAngularVelocity;
  }

  //public double getDisFromHub() {
  public FieldCentricFacingAngle servoToHub() {
    FieldCentricFacingAngle visionRequest = new FieldCentricFacingAngle();
    return visionRequest.withTargetDirection(new Rotation2d(getAngleToHub()));
  }

  private Translation2d getHubPose() {   
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
    //hubPose = new Translation2d(11.914324760437012, 4.033950328826904);

    if (alliance == Alliance.Blue) {
      hubPose = new Translation2d(4.622838497161865, 4.033950328826904);
    } else {
      hubPose = new Translation2d(11.914324760437012, 4.033950328826904);
    }

    return hubPose;
  }

  public double getDisFromHub() {
    hubPose = getHubPose();

    Translation2d ourPose = swerve.getPose().getTranslation();

    double eucDist = Math.sqrt(Math.pow(ourPose.getX() - hubPose.getX(), 2) + Math.pow(ourPose.getY() - hubPose.getY(), 2));    
    visionDist.set(eucDist);
    
    DogLog.log("dist", eucDist);
    
    return eucDist;
  }

  public double getAngleToHub() {
    hubPose = getHubPose();
    Translation2d ourPose = swerve.getPose().getTranslation();

    double xDist = (hubPose.getX() - ourPose.getX());
    double yDist = (hubPose.getY() - ourPose.getY());

    //tan(angle) opposite / adjacent = ∆y/∆x so angle = arctan(∆y/∆x)
    double angle = Math.atan2(yDist, xDist);

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      return (angle+Math.PI);
    }
    else {
      return (angle);
    }

    //return (angle + Math.PI);
  }


  @Override
  public void periodic() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        hubPose = new Translation2d(4.622838497161865, 4.033950328826904);
      } else {
        hubPose = new Translation2d(11.914324760437012, 4.033950328826904);
      }
    }

    Translation2d ourPose = swerve.getPose().getTranslation();

    double eucDist = Math.sqrt(Math.pow(ourPose.getX() - hubPose.getX(), 2) + Math.pow(ourPose.getY() - hubPose.getY(), 2));    
    visionDist.set(eucDist);
  }
}