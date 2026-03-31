// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;  
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import dev.doglog.DogLog;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Visabelle extends SubsystemBase {
  private double maxAngularRate;
  private SwerveOnTheseBows swerve;
  private Translation2d hubPose;
  
  
  public Visabelle(SwerveOnTheseBows drivetrain, double maxAngularRate) {
    this.swerve = drivetrain;
    this.maxAngularRate = maxAngularRate;
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

  private Translation2d getHubPose() {
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
    //FOR SIMULATION:
    //hubPose = Constants.Visabelle.BLUE_HUB_POSE;
    //hubPose = Constants.Visabelle.RED_HUB_POSE;

    if (alliance == Alliance.Blue) {
      hubPose = Constants.Visabelle.BLUE_HUB_POSE;
    } else {
      hubPose = Constants.Visabelle.RED_HUB_POSE;
    }

    return hubPose;
  }

  public double getDisFromHub() {
    hubPose = getHubPose();

    Translation2d ourPose = swerve.getPose().getTranslation();

    double eucDist = Math.hypot(ourPose.getX() - hubPose.getX(), ourPose.getY() - hubPose.getY());
    DogLog.log("Visabelle/dist from hub", eucDist);
    
    if (Constants.Visabelle.verboseLogging) {
      DogLog.log("Visabelle/target hub", hubPose);
    }
    
    return eucDist;
  }

  public double getAngleToHub() {
    hubPose = getHubPose();
    Translation2d ourPose = swerve.getPose().getTranslation();

    double xDist = (hubPose.getX() - ourPose.getX());
    double yDist = (hubPose.getY() - ourPose.getY());

    //tan(angle) opposite / adjacent = ∆y/∆x so angle = arctan(∆y/∆x)
    double angle = Math.atan2(yDist, xDist);

    if (Constants.Visabelle.verboseLogging) {
      DogLog.log("Visabelle/raw angle to hub", angle);
    }

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      angle += Math.PI;
    }

    DogLog.log("Visabelle/angle to hub", angle);
    return (angle);
  }

  public boolean isAligned() {
    return (getAngleToHub() <= 0.0872665); //5 degrees to radians
  }

  public BooleanSupplier getIsAligned() {
    return (() -> getAngleToHub() <= 0.0872665); //5 degrees to radians
  }

  public void configDashboard(ShuffleboardTab tab) {
    HttpCamera httpCamera1 = new HttpCamera("limelight-front", "http://10.19.67.14:5801/"); //http://10.19.67.202:5801/
    CameraServer.addCamera(httpCamera1);
    tab.add(httpCamera1).withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 0)
        .withSize(3, 2);

    HttpCamera httpCamera2 = new HttpCamera("limelight-back", "http://10.19.67.15:5801/"); //http://10.19.67.202:5801/
    CameraServer.addCamera(httpCamera2);
    tab.add(httpCamera2).withWidget(BuiltInWidgets.kCameraStream).withPosition(3, 0)
        .withSize(3, 2);

    tab.addBoolean("LL isAligned", this.getIsAligned())
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(7, 1)
        .withSize(1, 1);
  }

  @Override
  public void periodic() {
    hubPose = Constants.Visabelle.BLUE_HUB_POSE; // In case we're not connected yet
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        hubPose = Constants.Visabelle.BLUE_HUB_POSE;
        hubPose = Constants.Visabelle.BLUE_HUB_POSE;
      } else {
        hubPose = Constants.Visabelle.RED_HUB_POSE;
        hubPose = Constants.Visabelle.RED_HUB_POSE;
      }
    }

    this.getDisFromHub();
  }
}