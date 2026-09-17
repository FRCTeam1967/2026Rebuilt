// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisabelleUpdate extends SubsystemBase {
  /** Creates a new VisionUpdate. */

  // visibility dataType name;
  private SwerveOnTheseBows swerve;

  private boolean isTowerPoseSet = false;

  public static Pose2d towerPose = Constants.Visabelle.RED_TOWER; // Initialize to something

  LimelightHelpers.PoseEstimate mt2_front;
  LimelightHelpers.PoseEstimate mt2_back;
  double frontAmbiguity;
  double backAmbiguity;
  double deviation;

  public VisabelleUpdate(SwerveOnTheseBows swerve) {
    this.swerve = swerve;
  }

  /**
   * @param estimate the position estimate based on the limelight's calculations
   * @return if the new position update should be accepted as the robot's position
   */
  public boolean rejectUpdate(LimelightHelpers.PoseEstimate estimate) {
    if (estimate == null) {
      DogLog.log("VisabelleUpdate/reject reason", "no PoseEstimate");
        return true;
    }

    if (estimate.tagCount == 0) {
        DogLog.log("VisabelleUpdate/reject reason", "no PoseEstimate tag");
        return true;
    }

    if (estimate.avgTagDist > Constants.Visabelle.DIST_THRESHOLD) {
        DogLog.log("VisabelleUpdate/reject reason", "too far");
        return true;
    }

    if (estimate.rawFiducials.length >= 1) {
      if (estimate.rawFiducials[0].ambiguity > 0.8) {
        DogLog.log("VisabelleUpdate/reject reason", "ambiguity too high");
        return true;
      }
    }

    // angular velocity
    if (swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble() > 360.0) {
    DogLog.log("VisabelleUpdate/reject reason", "angular velocity too high");  
      return true;
    }

    // speed
    ChassisSpeeds speeds = swerve.getState().Speeds;

    double linearVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    if (linearVelocity > 5.0) { 
        DogLog.log("VisabelleUpdate/reject reason", "linear velocity too high"); 
        return true;    
    }

    return false;
  }

  /**
   * This method checks to see if any tag 
   * @return if Limelight can see a tag from either the back of the front
   */
  public boolean canSeeATag() { 
    mt2_front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    mt2_back = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

    return (mt2_front.tagCount > 0 || mt2_back.tagCount > 0);
  }
  
  /**
   * sets the first position upon enabling in teleop depending on what tags it sees
   */
  public void setFirstVisionPose() {
    mt2_front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    mt2_back = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

    if (mt2_front.tagCount > 0 || mt2_back.tagCount > 0) {
      if (mt2_front.tagCount > 0) {
        swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.0,0.0,9999999));

        swerve.addVisionMeasurement(
          mt2_front.pose,
          mt2_front.timestampSeconds);
      }

      if (mt2_back.tagCount > 0) {
        swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.0,0.0,9999999));

        swerve.addVisionMeasurement(
          mt2_back.pose,
          mt2_back.timestampSeconds);
      }
    }
  }

  /**
   * set standard deviations based on ambiguity (the lower the ambiguity, the more we trust it) 
   * @param estimate an estimation of the position based on vision calculation
   * @param isFront whether the tag is seen from the front limelight or not
  */
  public void setStandardDevs(LimelightHelpers.PoseEstimate estimate, boolean isFront) {
    String name = isFront ? "limelight-front" : "limelight-back";
    double tagCount = LimelightHelpers.getTargetCount(name);
    double distance = estimate.rawFiducials[0].distToCamera;
    double ambiguity = estimate.rawFiducials[0].ambiguity;

    double deviation = 0.75 * distance * Math.pow(ambiguity, 2);

    DogLog.log("deviation", deviation);
    DogLog.log("LL name", name);

    swerve.setVisionMeasurementStdDevs(VecBuilder.fill(deviation, deviation,9999999));
  }

  /**
   * gets the deviation of the robot based on the ambiguity and distance from the tag
   * @return deviation of the robot
   * @param estimate an estimation of the position based on vision calculation
   * @param isFront whether the tag is seen from the front limelight or not
   */
  public double getDeviation(LimelightHelpers.PoseEstimate estimate, boolean isFront) {
      String name = isFront ? "limelight-front" : "limelight-back";
      double tagCount = LimelightHelpers.getTargetCount(name);
      double distance = estimate.rawFiducials[0].distToCamera;
      double ambiguity = estimate.rawFiducials[0].ambiguity;

      double deviation = 0.75 * distance * Math.pow(ambiguity, 2);

      return deviation;
  }

  /**
   * returns whether or not the back or front limelight's positions are close to tag
   * @param frontEstimate the front estimate for distance between the tag and the front limelight
   * @param backEstimate the back estimate for distance between the tag and the back limelight
   * @return whether or not the positions are close to the tags, either front or back
   */
  public boolean posesAreClose(LimelightHelpers.PoseEstimate frontEstimate, LimelightHelpers.PoseEstimate backEstimate) {
    Pose2d frontPose = frontEstimate.pose;
    Pose2d backPose = backEstimate.pose;

    double eucDist = frontPose.getTranslation().getDistance(backPose.getTranslation());

    double MAX_DISTANCE = 0.33655; // 13.25 in (half the robot)

    return eucDist <= MAX_DISTANCE;
  }

  @Override
  public void periodic() {
    if (!isTowerPoseSet){
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            towerPose = Constants.Visabelle.RED_TOWER;
        } else {
            towerPose = Constants.Visabelle.BLUE_TOWER;
        }
        isTowerPoseSet = true;
      }
    }

    LimelightHelpers.SetRobotOrientation("limelight-front", (swerve.getPigeon2().getRotation2d().getDegrees()), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-back", (swerve.getPigeon2().getRotation2d().getDegrees()), 0, 0, 0, 0, 0);

    mt2_front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    mt2_back = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
    
    if (mt2_front.rawFiducials.length >= 1) {
      frontAmbiguity = mt2_front.rawFiducials[0].ambiguity;
    } else {
      frontAmbiguity = 9999999;
    }
    
    if (mt2_back.rawFiducials.length >= 1) {
      backAmbiguity = mt2_back.rawFiducials[0].ambiguity;
    } else {
      backAmbiguity = 9999999;
    }

    DogLog.log("VisabelleUpdate/front limelight pose", mt2_front.pose);
    DogLog.log("VisabelleUpdate/back limelight pose", mt2_back.pose);
    DogLog.log("VisabelleUpdate/front ambiguity", frontAmbiguity);
    DogLog.log("VisabelleUpdate/back ambiguity", backAmbiguity);

    //logging all the tags we can see
    if (mt2_front != null && mt2_front.tagCount > 0) {
      long tagArray[] = new long[mt2_front.tagCount];
      int idx = 0;
      for (RawFiducial fiducial: mt2_front.rawFiducials) {
        tagArray[idx++] = fiducial.id;
      }
      DogLog.log("Front fiducials", tagArray);
    }

    if (mt2_back != null && mt2_back.tagCount > 0) {
      long tagArray[] = new long[mt2_back.tagCount];
      int idx = 0;
      for (RawFiducial fiducial: mt2_back.rawFiducials) {
        tagArray[idx++] = fiducial.id;
      }
      DogLog.log("Back fiducials", tagArray);
    }

    // accept only front
    if (!rejectUpdate(mt2_front) && rejectUpdate(mt2_back)) {
      setStandardDevs(mt2_front, true);

      swerve.addVisionMeasurement(
        mt2_front.pose,
        mt2_front.timestampSeconds);
    }

    // accept only back
    else if (!rejectUpdate(mt2_back) && rejectUpdate(mt2_front)){
      setStandardDevs(mt2_back, false);

      swerve.addVisionMeasurement(
        mt2_back.pose,
        mt2_back.timestampSeconds);
    }

    // accept both
    else if (!rejectUpdate(mt2_front) && !rejectUpdate(mt2_back)){           

      if (posesAreClose(mt2_front, mt2_back)) {
        // add front
        setStandardDevs(mt2_front, true);
        swerve.addVisionMeasurement(
          mt2_front.pose, 
          mt2_front.timestampSeconds);

        // add back
        setStandardDevs(mt2_back, false);        
        swerve.addVisionMeasurement(
          mt2_back.pose, 
          mt2_back.timestampSeconds);

      }

      else {
        if (getDeviation(mt2_front, true) < getDeviation(mt2_back, false)) {
          setStandardDevs(mt2_front, true);
          swerve.addVisionMeasurement(
            mt2_front.pose, 
            mt2_front.timestampSeconds);
        }
        else {
          setStandardDevs(mt2_back, true);
          swerve.addVisionMeasurement(
            mt2_back.pose, 
            mt2_back.timestampSeconds);
        }
      }
    }
  }
}