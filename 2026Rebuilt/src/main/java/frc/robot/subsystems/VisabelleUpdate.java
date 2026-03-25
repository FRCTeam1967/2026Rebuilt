// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
//import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.LimelightHelpers;
// import frc.robot.LimelightHelpers.LimelightResults;
// import edu.wpi.first.wpilibj.Timer;

public class VisabelleUpdate extends SubsystemBase {
  /** Creates a new VisionUpdate. */
  public static Pose2d towerPose;

  // visibility dataType name;
  private SwerveOnTheseBows swerve;

  // number of accepted pose estimates
  private int frontLLEstimatecount = 0;
  private int backLLEstimatecount = 0;

  //private static final double AREA_THRESHOLD = 0.1;
  //private static final double DIST_THRESHOLD = 4.572; // 15 ft in meters

  private static final Pose2d RED_TOWER = new Pose2d(15.421048, 3.432656, new Rotation2d(Math.PI));
  private static final Pose2d BLUE_TOWER = new Pose2d(1.092, 4.61, new Rotation2d(0.0));
  //private static final Vector<N3> VISION_STD_DEVS = VecBuilder.fill(.7, .7, 9999999);

  public static Pose2d towerPose = RED_TOWER; // Initialize to something

  LimelightHelpers.PoseEstimate mt2_front;
  LimelightHelpers.PoseEstimate mt2_back;
  double frontAmbiguity;
  double backAmbiguity;

  // used for timestamp of previous periodic loop
  // double frontTimestamp;
  // double backTimestamp;

  public VisabelleUpdate(SwerveOnTheseBows swerve) {
    this.swerve = swerve;
  }

  public boolean rejectUpdate(LimelightHelpers.PoseEstimate estimate) {
    if (estimate == null)
        return true;

    if (estimate.tagCount == 0)
        return true;

    // if (estimate.tagCount == 1 && estimate.avgTagDist < DIST_THRESHOLD)
    //     return true;

    if (estimate.rawFiducials.length >= 1) {
      if (estimate.rawFiducials[0].ambiguity > 0.9)
        return true;
    }
    
    return false;
  }

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


  @Override
  public void periodic() {
    // TODO: see if we can move this to disabled periodic (bc why not?)
    if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            towerPose = RED_TOWER;
        } else {
            towerPose = BLUE_TOWER;
        }
    }

    LimelightHelpers.SetRobotOrientation("limelight-front", (swerve.getPigeon2().getRotation2d().getDegrees()), 0, 0, 0, 0, 0);

    // // front timestamp accounting for latency
    // var front_t2d = LimelightHelpers.getT2DArray("limelight-front");
    // var frontTargetLatency = front_t2d[2];
    // var frontCaptureLatency = front_t2d[3];

    // var frontLatency = frontTargetLatency + frontCaptureLatency;
    // var frontLatencySeconds = frontLatency / 1000.0;
    // var frontTimestamp = Timer.getFPGATimestamp() - frontLatencySeconds;

    // // back timestamp accounting for latency
    // var back_t2d = LimelightHelpers.getT2DArray("limelight-back");
    // var backTargetLatency = back_t2d[2];
    // var backCaptureLatency = back_t2d[3];

    // var backLatency = backTargetLatency + backCaptureLatency;
    // var backLatencySeconds = backLatency / 1000.0;
    // var backTimestamp = Timer.getFPGATimestamp() - backLatencySeconds;

    mt2_front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    mt2_back = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
    
    if (mt2_front.rawFiducials.length >= 1) {
      frontAmbiguity = mt2_front.rawFiducials[0].ambiguity;
    } else {
      frontAmbiguity = 0.5;
    }
    
    if (mt2_back.rawFiducials.length >= 1) {
      backAmbiguity = mt2_back.rawFiducials[0].ambiguity;
    } else {
      backAmbiguity = 0.5;
    }

    //LimelightHelpers.PoseEstimate chosenPoseEstimate = null;

    DogLog.log("VisabelleUpdate/front limelight pose", mt2_front.pose);
    DogLog.log("VisabelleUpdate/back limelight pose", mt2_back.pose);

    // choose based on smaller distance
    // if (!rejectUpdate(mt2_front) && rejectUpdate(mt2_back)) {
    //   chosenPoseEstimate = mt2_front;
    //   DogLog.log("front count", frontLLcount++);
    // }
    // else if (!rejectUpdate(mt2_back) && rejectUpdate(mt2_front)) {
    //   chosenPoseEstimate = mt2_back;
    //   DogLog.log("back count", backLLcount++);
    // }
    // else if (!rejectUpdate(mt2_front) && !rejectUpdate(mt2_back)){           
    //   if (mt2_front.avgTagDist > mt2_back.avgTagDist) {
    //     chosenPoseEstimate = mt2_back;
    //     DogLog.log("back count", backLLcount++);
    //   }
    //   else {
    //     chosenPoseEstimate = mt2_front;
    //     DogLog.log("front count", frontLLcount++);
    //   }
    // }
    
    // reset pose to next raw vision estimate after 5 seconds of not seeing a tag
    // if ((mt2_front.timestampSeconds > (prevFrontTimestamp + 5)) && (mt2_back.timestampSeconds > (Timer.getFPGATimestamp() + 5))) {
    //   swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.0,0.0,9999999));

    //   if (mt2_front.tagCount > 0) {
    //     swerve.addVisionMeasurement(
    //         mt2_front.pose,
    //         mt2_front.timestampSeconds);
    //   }

    //   if (mt2_back.tagCount > 0) {
    //     swerve.addVisionMeasurement(
    //         mt2_back.pose,
    //         mt2_back.timestampSeconds);
    //   }

    //   prevFrontTimestamp = mt2_front.timestampSeconds;
    //   prevBackTimestamp = mt2_front.timestampSeconds;
    // }

    // else {
    //   frontTimestamp = mt2_front.timestampSeconds;
    //   backTimestamp = mt2_front.timestampSeconds;

    // accept only front
    if (!rejectUpdate(mt2_front) && rejectUpdate(mt2_back)) {
      swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,9999999)); //TODO: tune?

      swerve.addVisionMeasurement(
        mt2_front.pose,
        mt2_front.timestampSeconds);
      
      DogLog.log("VisabelleUpdate/front upd count", frontLLEstimatecount++);
    }

    // accept only back
    else if (!rejectUpdate(mt2_back) && rejectUpdate(mt2_front)) {
      swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,9999999));

      swerve.addVisionMeasurement(
        mt2_back.pose,
        mt2_back.timestampSeconds);

      DogLog.log("VisabelleUpdate/back upd count", backLLEstimatecount++);
    }

    // accept both
    else if (!rejectUpdate(mt2_front) && !rejectUpdate(mt2_back)){           
      //if (mt2_front.tagCount > mt2_back.tagCount) {

      // trust front more
      if (frontAmbiguity < backAmbiguity) {
        // add front
        swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.5,0.5,9999999));
        
        swerve.addVisionMeasurement(
          mt2_front.pose,
          mt2_front.timestampSeconds);

        // add back
        swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.9,0.9,9999999)); //TODO: what is the range of the values?
        
        swerve.addVisionMeasurement(
          mt2_back.pose,
          mt2_back.timestampSeconds);
        
        DogLog.log("VisabelleUpdate/accept both", "Front 0.7, Back 999");
        DogLog.log("VisabelleUpdate/front upd count", frontLLEstimatecount++);
        DogLog.log("VisabelleUpdate/back upd count", backLLEstimatecount++);
      }

      // trust back more
      // else if (mt2_back.tagCount > mt2_front.tagCount) {
      else if (backAmbiguity < frontAmbiguity) {
        // front
        swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.9,0.9,9999999));
        
        swerve.addVisionMeasurement(
          mt2_front.pose,
          mt2_front.timestampSeconds);

        // back
        swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.5,0.5,9999999));
        
        swerve.addVisionMeasurement(
          mt2_back.pose,
          mt2_back.timestampSeconds);
        
        DogLog.log("VisabelleUpdate/acc both", "Front 999, Back 0.7");
        DogLog.log("VisabelleUpdate/front upd count", frontLLEstimatecount++);
        DogLog.log("VisabelleUpdate/back upd count", backLLEstimatecount++);
      }

      // same amount of tags
      else if (frontAmbiguity == backAmbiguity) {
        swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,9999999));
        swerve.addVisionMeasurement(
          mt2_front.pose,
          mt2_front.timestampSeconds);

        DogLog.log("VisabelleUpdate/front upd count", frontLLEstimatecount++);
      }
      //}
    }

    // if (chosenPoseEstimate != null) {
    //   DogLog.log("VisabelleUpdate/chosen update", chosenPoseEstimate.pose);
    //   DogLog.log("VisabelleUpdate/vision update accepted", true);
    //   swerve.setVisionMeasurementStdDevs(VISION_STD_DEVS);
          
    //   swerve.addVisionMeasurement(
    //     chosenPoseEstimate.pose,
    //     chosenPoseEstimate.timestampSeconds);

    //     //chosenPoseEstimate = null;
    //   }
    // else {
    //   DogLog.log("VisabelleUpdate/vision update accepted", false);
    // } 
  }
}