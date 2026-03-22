// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class VisabelleUpdate extends SubsystemBase {
  /** Creates a new VisionUpdate. */

  // visibility dataType name;
  private SwerveOnTheseBows swerve;

  private int frontLLcount = 0;
  private int backLLcount = 0;

  private static final double AREA_THRESHOLD = 0.1;

  private static final Pose2d RED_TOWER = new Pose2d(15.421048, 3.432656, Rotation2d.kPi);
  private static final Pose2d BLUE_TOWER = new Pose2d(1.092, 4.61, Rotation2d.kZero);
  private static final Vector<N3> VISION_STD_DEVS = VecBuilder.fill(.7, .7, 9999999);

  public static Pose2d towerPose = RED_TOWER; // Initialize to something

  public VisabelleUpdate(SwerveOnTheseBows swerve) {
    this.swerve = swerve;
  }

  public boolean rejectUpdate(LimelightHelpers.PoseEstimate estimate) {
    if (estimate == null)
        return true;

    if (estimate.tagCount == 0)
        return true;

    if (estimate.tagCount == 1 && estimate.avgTagArea < AREA_THRESHOLD)
        return true;

    return false;
  }

  // TODO: add conditional to add either front or back limelight
  public void setFirstVisionPose() {
    LimelightHelpers.PoseEstimate mt2_front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    LimelightHelpers.PoseEstimate mt2_back = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

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

  public Pose3d targetPose = new Pose3d();
  public LimelightResults results = new LimelightResults();

  private boolean isInRange = false;

  public boolean getInRange() {
    return isInRange;
  }

  public void setInRangeTrue() {
    isInRange = true;
  }

  public void setInRangeFalse() {
    isInRange = false;
  }

  @Override
  public void periodic() {
    if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            towerPose = RED_TOWER;
        } else {
            towerPose = BLUE_TOWER;
        }
    }

    if (Constants.Visabelle.verboseLogging) {
      DogLog.log("VisabelleUpdate/isInRange", isInRange);
    }
    
    LimelightHelpers.SetRobotOrientation("limelight-front", (swerve.getPigeon2().getRotation2d().getDegrees()), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-back", (swerve.getPigeon2().getRotation2d().getDegrees()), 0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate mt2_front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    LimelightHelpers.PoseEstimate mt2_back = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
    LimelightHelpers.PoseEstimate chosenPoseEstimate = null;


    DogLog.log("VisabelleUpdate/front limelight pose", mt2_front.pose);
    DogLog.log("VisabelleUpdate/back limelight pose", mt2_back.pose);


    if (!rejectUpdate(mt2_front) && rejectUpdate(mt2_back)) {
      chosenPoseEstimate = mt2_front;
      DogLog.log("front count", frontLLcount++);
    }
    else if (!rejectUpdate(mt2_back) && rejectUpdate(mt2_front)) {
      chosenPoseEstimate = mt2_back;
      DogLog.log("back count", backLLcount++);
    }
    else if (!rejectUpdate(mt2_front) && !rejectUpdate(mt2_back)){           
      if (mt2_front.avgTagDist > mt2_back.avgTagDist) {
        chosenPoseEstimate = mt2_back;
        DogLog.log("back count", backLLcount++);
      }
      else {
        chosenPoseEstimate = mt2_front;
        DogLog.log("front count", frontLLcount++);
      }
    }

    if (chosenPoseEstimate != null) {
      DogLog.log("VisabelleUpdate/chosen update", chosenPoseEstimate.pose);
      DogLog.log("VisabelleUpdate/vision update accepted", true);
      swerve.setVisionMeasurementStdDevs(VISION_STD_DEVS);
          
      swerve.addVisionMeasurement(
        chosenPoseEstimate.pose,
        chosenPoseEstimate.timestampSeconds);

        //chosenPoseEstimate = null;
      }
    else {
      DogLog.log("VisabelleUpdate/vision update accepted", false);
    } 
  }
}