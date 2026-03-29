// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import java.util.ArrayList;

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
import frc.robot.LimelightHelpers.RawFiducial;

public class VisabelleUpdate extends SubsystemBase {
  /** Creates a new VisionUpdate. */

  // visibility dataType name;
  private SwerveOnTheseBows swerve;

  // number of accepted pose estimates
  private int frontLLEstimatecount = 0;
  private int backLLEstimatecount = 0;

  //private static final double AREA_THRESHOLD = 0.1;
  //private static final double DIST_THRESHOLD = 4.572; // 15 ft in meters

  private boolean isTowerPoseSet = false;

  //private int[] validIDs = new int[16];

  //private static final double AREA_THRESHOLD = 0.1;
  //private static final double DIST_THRESHOLD = 4.572; // 15 ft in meters

  // private static final Pose2d RED_TOWER = new Pose2d(15.421048, 3.432656, new Rotation2d(Math.PI));
  // private static final Pose2d BLUE_TOWER = new Pose2d(1.092, 4.61, new Rotation2d(0.0));
  //private static final Vector<N3> VISION_STD_DEVS = VecBuilder.fill(.7, .7, 9999999);

  public static Pose2d towerPose = Constants.Visabelle.RED_TOWER; // Initialize to something

  LimelightHelpers.PoseEstimate mt2_front;
  LimelightHelpers.PoseEstimate mt2_back;
  double frontAmbiguity;
  double backAmbiguity;
  double deviation;

  // used for timestamp of previous periodic loop
  // double frontTimestamp;
  // double backTimestamp;

  public VisabelleUpdate(SwerveOnTheseBows swerve) {
    this.swerve = swerve;
  }

  public boolean rejectUpdate(LimelightHelpers.PoseEstimate estimate) {
    if (estimate == null)
        return true;

    //TODO: this should reject all if we don't see anything, test if that's the case
    if (estimate.tagCount == 0)
        return true;

    if (estimate.avgTagDist > Constants.Visabelle.DIST_THRESHOLD)
        return true;

    if (estimate.rawFiducials.length >= 1) {
      if (estimate.rawFiducials[0].ambiguity > 0.8) //TODO: according to LL chief delphi, use 0.9- according to jonah from mechanical advantage (advantage scope guy), use something lower like 0.4
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

  // set standard deviations based on ambiguity (the lower the ambiguity, the more we trust it)
  public void setStandardDevs(LimelightHelpers.PoseEstimate estimate, boolean isFront) {
    String name = isFront ? "limelight-front" : "limelight-back";
    double tagCount = LimelightHelpers.getTargetCount(name);
    double distance = estimate.rawFiducials[0].distToCamera;
    double ambiguity = estimate.rawFiducials[0].ambiguity;

    // double distanceScale = Math.pow(distance, 2);
    // double tagScale = 1.0 / tagCount;
    // double ambiguityScale = 1.0 + 5.0 * ambiguity; // 5.0 is a magic number here
    // double scale = distanceScale * tagScale * ambiguityScale;
    // double deviation = 0.01 * scale;

    //TODO: test these derivations in order, see if they are useful
    double deviation = 6.0 * Math.pow(ambiguity, 3); 
        //verify examples: 0.1	>> 0.006; 0.3	>> 0.162; 0.5 >> 0.75; 0.7 >> 2.06; 1.0	>> 6.0

    //2) double deviation = 0.75 * distance * Math.pow(ambiguity, 2);
        //(dist >> 1) 0.1 >> 0.0075; 0.3 >> 0.0675; 0.5 >> 0.1875; 0.7 >> 0.3675
        //(dist >> 2) 0.1 >> 0.015; 0.3 >> 0.135; 0.5 >> 0.375; 0.7 >> 0.735
        //(dist >> 4) 0.1 >> 0.03; 0.3 >> 0.27; 0.5 >> 0.75; 0.7 >> 1.47

    //3) double deviation = 0.75 * Math.pow(distance, 2) * Math.pow(ambiguity, 2);
        //(dist >> 1) 0.1 >> 0.0075; 0.3 >> 0.0675; 0.5 >> 0.1875; 0.7 >> 0.3675
        //(dist >> 2) 0.1 >> 0.03; 0.3 >> 0.27; 0.5 >> 0.75; 0.7 >> 1.47
        //(dist >> 4) 0.1 >> 0.12; 0.3 >> 1.08; 0.5 >> 3.0; 0.7 >> 5.88


    //4) double deviation = 0.5 * Math.pow(distance, 2) * Math.pow(ambiguity, 2) * (1.0 / Math.sqrt(tagCount));
        //(dist >> 2, tags >> 1) 0.1 >> 0.02; 0.3 >> 0.18; 0.5 >> 0.5; 0.7 >> 0.98
        //(dist >> 2, tags >> 4) 0.1 >> 0.01; 0.3 >> 0.09; 0.5 >> 0.25; 0.7 >> 0.49
        //(dist >> 4, tags >> 1) 0.1 >> 0.08; 0.3 >> 0.72; 0.5 >> 2.0; 0.7 >> 3.92
        //(dist >> 4, tags >> 4) 0.1 >> 0.04; 0.3 >> 0.36; 0.5 >> 1.0; 0.7 >> 1.96

    DogLog.log("deviation", deviation);
    DogLog.log("LL name", name);

    swerve.setVisionMeasurementStdDevs(VecBuilder.fill(deviation, deviation,9999999));
  }

  @Override
  public void periodic() {
    // TODO: see if we can move this to disabled periodic (bc why not?)
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

    //LimelightHelpers.SetFiducialIDFiltersOverride("limelight-front", validIDs);
    //LimelightHelpers.SetFiducialIDFiltersOverride("limelight-back", validIDs);
    
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

    //LimelightHelpers.PoseEstimate chosenPoseEstimate = null;
    DogLog.log("VisabelleUpdate/front limelight pose", mt2_front.pose);
    DogLog.log("VisabelleUpdate/back limelight pose", mt2_back.pose);
    DogLog.log("VisabelleUpdate/front avgtagdist", mt2_front.avgTagDist);
    DogLog.log("VisabelleUpdate/back avgtagdist", mt2_back.avgTagDist);
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
      setStandardDevs(mt2_front, true);

      swerve.addVisionMeasurement(
        mt2_front.pose,
        mt2_front.timestampSeconds);
      
      DogLog.log("VisabelleUpdate/front upd count", frontLLEstimatecount++);
    }

    // accept only back
    else if (!rejectUpdate(mt2_back) && rejectUpdate(mt2_front)){
      setStandardDevs(mt2_back, false);

      swerve.addVisionMeasurement(
        mt2_back.pose,
        mt2_back.timestampSeconds);

      DogLog.log("VisabelleUpdate/back upd count", backLLEstimatecount++);
    }

    // accept both
    else if (!rejectUpdate(mt2_front) && !rejectUpdate(mt2_back)){           
      //if (mt2_front.tagCount > mt2_back.tagCount) {

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
        
        DogLog.log("VisabelleUpdate/accept both", "Front 0.7, Back 999");
        DogLog.log("VisabelleUpdate/front upd count", frontLLEstimatecount++);
        DogLog.log("VisabelleUpdate/back upd count", backLLEstimatecount++);
    }
  }
}