// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

import java.util.Optional;

import com.ctre.phoenix6.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class VisabelleUpdate extends SubsystemBase {
  /** Creates a new VisionUpdate. */

  // visibility dataType name;
  private SwerveOnTheseBows swerve;

  private final StructPublisher<Pose2d> frontlightPublisher;
  private final StructPublisher<Pose2d> backlightPublisher;

  private long odometryUpdates = 0;
  private long odometryDiscards = 0;
  IntegerPublisher updatePublisher;
  IntegerPublisher discardPublisher;

  DoublePublisher LLtimestamp;
  DoublePublisher fpgaTimestamp;
  DoublePublisher LLtoFPGA;

  private static final double MAX_ANGULAR_VELOCITY = 720; // TODO: change
  private static final double AREA_THRESHOLD = 0.1;

  public static Pose2d towerPose;
  public static LimelightHelpers.PoseEstimate chosenPoseEstimate;

  public VisabelleUpdate(SwerveOnTheseBows swerve) {
    this.swerve = swerve;

    frontlightPublisher = NetworkTableInstance.getDefault().getTable("limelight-front").getStructTopic("Front Limelight Pose", Pose2d.struct).publish();
    backlightPublisher = NetworkTableInstance.getDefault().getTable("limelight-back").getStructTopic("Back Limelight Pose", Pose2d.struct).publish();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("odometryStats");

    updatePublisher = table.getIntegerTopic("acceptedUpdates").publish();
    discardPublisher = table.getIntegerTopic("rejectedUpdates").publish();

    LLtimestamp = table.getDoubleTopic("limelight timestamp").publish();
    fpgaTimestamp = table.getDoubleTopic("fpga timestamp").publish();
    LLtoFPGA = table.getDoubleTopic("LL converted to fpga").publish();

    Optional<Alliance> ally = DriverStation.getAlliance();

    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            towerPose = new Pose2d(15.421048, 3.432656, new Rotation2d(Math.PI));
            DogLog.log("Tower Pose: ", towerPose);
        } else {
            towerPose = new Pose2d(1.092, 4.61, new Rotation2d(0.0));
            DogLog.log("Tower Pose: ", towerPose);
        }
    } else {
        towerPose = new Pose2d(15.421048, 3.432656, new Rotation2d(Math.PI));
        DogLog.log("Default Red: ", towerPose);
    }
  }

  public boolean rejectUpdate(LimelightHelpers.PoseEstimate estimate) {
    if (estimate == null)
        return true;

    if (estimate.tagCount == 0)
        return true;

    // Angular velocity is too high to have accurate vision
    //if (Math.abs(angularVelocity) > MAX_ANGULAR_VELOCITY)
    //    return true;

    if (estimate.tagCount == 1 && estimate.avgTagArea < AREA_THRESHOLD)
        return true;

    return false;
  }

  // TODO: add conditional to add either front or back limelight
  public void setFirstVisionPose() {
    LimelightHelpers.PoseEstimate mt2_front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    LimelightHelpers.PoseEstimate mt2_back = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

    if (!rejectUpdate(mt2_front) && rejectUpdate(mt2_back)) {
      chosenPoseEstimate = mt2_front;
      frontlightPublisher.set(mt2_front.pose);
    }
    else if (!rejectUpdate(mt2_back) && rejectUpdate(mt2_front)) {
      chosenPoseEstimate = mt2_back;
      backlightPublisher.set(mt2_back.pose);
    }
    else if (!rejectUpdate(mt2_front) && !rejectUpdate(mt2_back)){           
      if (mt2_front.avgTagDist > mt2_back.avgTagDist) {
        chosenPoseEstimate = mt2_back;
      }
      else {
        chosenPoseEstimate = mt2_front;
      }
      frontlightPublisher.set(mt2_front.pose);
      backlightPublisher.set(mt2_back.pose);
    }
    else {
        chosenPoseEstimate = null;
        return;
    }

    swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.0,0.0,9999999));

    swerve.addVisionMeasurement(
      chosenPoseEstimate.pose,
      Utils.fpgaToCurrentTime(chosenPoseEstimate.timestampSeconds));
  }

  private boolean disableVision = false;
  // private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  public NetworkTable limelightTable, limelightOdometryTable;
  public LimelightTarget_Fiducial limelightTargetFiducial = new LimelightTarget_Fiducial();

  public Pose3d targetPose = new Pose3d();
  public LimelightResults results = new LimelightResults();

  //Limelight Updating Values
  private double xAlignmentOffset, yAlignmentOffset, zAlignmentOffset, vAlignmentCheck;
  private double xOdometryOffset, yOdometryOffset, zOdometryOffset;
  private ChassisSpeeds alignSpeed;

  private boolean isInRange = false;

  public boolean isVisionDisabled(){
    return disableVision;
  }

  public boolean getInRange() {
    return isInRange;
  }

  public void setInRangeTrue() {
    isInRange = true;
  }

  public void setInRangeFalse() {
    isInRange = false;
  }

  public double getTXAlignmentOffset() {
    return xAlignmentOffset;
  }

  public double getAlignmentCheck() {
    return vAlignmentCheck;
  }

  @Override
  public void periodic() {
    if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            towerPose = new Pose2d(15.421048, 3.432656, new Rotation2d(Math.PI));
            DogLog.log("Tower Pose: ", towerPose);
        } else {
            towerPose = new Pose2d(1.092, 4.61, new Rotation2d(0.0));
            DogLog.log("Tower Pose: ", towerPose);
        }
    }
    
    LimelightHelpers.SetRobotOrientation("limelight-front", (swerve.getPigeon2().getRotation2d().getDegrees()), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-back", (swerve.getPigeon2().getRotation2d().getDegrees()), 0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate mt2_front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    LimelightHelpers.PoseEstimate mt2_back = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

    // //double angularVelocity = swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
    
    // if (!rejectUpdate(mt2_front) && rejectUpdate(mt2_back)) {
    //   chosenPoseEstimate = mt2_front;
    //   frontlightPublisher.set(mt2_front.pose);
    // }
    // else if (!rejectUpdate(mt2_back) && rejectUpdate(mt2_front)) {
    //   chosenPoseEstimate = mt2_back;
    //   backlightPublisher.set(mt2_back.pose);
    // }
    // else if (!rejectUpdate(mt2_front) && !rejectUpdate(mt2_back)){           
    //   if (mt2_front.avgTagDist > mt2_back.avgTagDist) {
    //     chosenPoseEstimate = mt2_back;
    //   }
    //   else {
    //     chosenPoseEstimate = mt2_front;
    //   }
    //   frontlightPublisher.set(mt2_front.pose);
    //   backlightPublisher.set(mt2_back.pose);
    // }

    // if (chosenPoseEstimate != null) {
    //   swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      
    //   swerve.addVisionMeasurement(
    //       chosenPoseEstimate.pose,
    //       chosenPoseEstimate.timestampSeconds);

    //   updatePublisher.set(++odometryUpdates);
    //   //chosenPoseEstimate = null;
    // }
    // else {
    //   discardPublisher.set(++odometryDiscards);
    // }  
    
    // if (chosenPoseEstimate != null) {
    //   LLtimestamp.set(chosenPoseEstimate.timestampSeconds);
    //   fpgaTimestamp.set(Utils.getCurrentTimeSeconds());
    //   LLtoFPGA.set(Utils.fpgaToCurrentTime(chosenPoseEstimate.timestampSeconds));
    // }

    if (mt2_front.tagCount > 0 || mt2_back.tagCount > 0) {
      swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      
      if (mt2_front.tagCount > 0) {
        swerve.addVisionMeasurement(
          mt2_front.pose,
          mt2_front.timestampSeconds);
      }

      if (mt2_back.tagCount > 0) {
        swerve.addVisionMeasurement(
          mt2_back.pose,
          mt2_back.timestampSeconds);
      }
      updatePublisher.set(++odometryUpdates);
      //chosenPoseEstimate = null;
    }
    else {
      discardPublisher.set(++odometryDiscards);
    }  

  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.LimelightHelpers;
// import frc.robot.LimelightHelpers.LimelightResults;
// import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

// import com.ctre.phoenix6.Utils;

// import dev.doglog.DogLog;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.networktables.DoublePublisher;
// import edu.wpi.first.networktables.IntegerPublisher;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StructPublisher;
// import frc.robot.subsystems.*;

// public class VisabelleUpdate extends SubsystemBase {
//   /** Creates a new VisionUpdate. */
//   public static Pose2d towerPose;

//   // visibility dataType name;
//   private SwerveOnTheseBows swerve;

//   private final StructPublisher<Pose2d> limelightPublisher;

//   private long odometryUpdates = 0;
//   private long odometryDiscards = 0;
//   IntegerPublisher updatePublisher;
//   IntegerPublisher discardPublisher;

//   DoublePublisher LLtimestamp;
//   DoublePublisher fpgaTimestamp;
//   DoublePublisher LLtoFPGA;

//   //public static Pose2d towerpose;

//   public VisabelleUpdate(SwerveOnTheseBows swerve) {
//     this.swerve = swerve;

//     limelightPublisher = NetworkTableInstance.getDefault().getTable("limelight-front").getStructTopic("Limelight Pose", Pose2d.struct).publish();

//     NetworkTableInstance inst = NetworkTableInstance.getDefault();
//     NetworkTable table = inst.getTable("odometryStats");

//     updatePublisher = table.getIntegerTopic("acceptedUpdates").publish();
//     discardPublisher = table.getIntegerTopic("rejectedUpdates").publish();


//     LLtimestamp = table.getDoubleTopic("limelight timestamp").publish();
//     fpgaTimestamp = table.getDoubleTopic("fpga timestamp").publish();
//     LLtoFPGA = table.getDoubleTopic("LL converted to fpga").publish();

//     Pose2d towerPose = new Pose2d(15.421048, 3.432656, new Rotation2d(Math.PI));
//     DogLog.log("Tower Pose: ", towerPose);
//   }

//   public void setFirstVisionPose() {
//     LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");

//     swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.0,0.0,9999999));

//     swerve.addVisionMeasurement(
//       mt2.pose,
//       Utils.fpgaToCurrentTime(mt2.timestampSeconds));
//   }

//   private boolean disableVision = false;
//   // private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

//   public NetworkTable limelightTable, limelightOdometryTable;
//   public LimelightTarget_Fiducial limelightTargetFiducial = new LimelightTarget_Fiducial();

//   public Pose3d targetPose = new Pose3d();
//   public LimelightResults results = new LimelightResults();

//   //Limelight Updating Values
//   private double xAlignmentOffset, yAlignmentOffset, zAlignmentOffset, vAlignmentCheck;
//   private double xOdometryOffset, yOdometryOffset, zOdometryOffset;
//   private ChassisSpeeds alignSpeed;

//   private boolean isInRange = false;

//   public boolean isVisionDisabled(){
//     return disableVision;
//   }

//   public boolean getInRange() {
//     return isInRange;
//   }

//   public void setInRangeTrue() {
//     isInRange = true;
//   }

//   public void setInRangeFalse() {
//     isInRange = false;
//   }

//   public double getTXAlignmentOffset() {
//     return xAlignmentOffset;
//   }

//   public double getAlignmentCheck() {
//     return vAlignmentCheck;
//   }


//   @Override
//   public void periodic() {
//     if (DriverStation.getAlliance().isPresent()) {
//         if (DriverStation.getAlliance().get() == Alliance.Red) {
//             towerPose = new Pose2d(15.421048, 3.432656, new Rotation2d(Math.PI));
//             DogLog.log("Tower Pose: ", towerPose);
//         } else {
//             towerPose = new Pose2d(1.092, 4.61, new Rotation2d(0.0));
//             DogLog.log("Tower Pose: ", towerPose);
//         }
//     }
    
//     boolean doRejectUpdate = false;
//     LimelightHelpers.SetRobotOrientation("limelight-front", (swerve.getPigeon2().getRotation2d().getDegrees()), 0, 0, 0, 0, 0);

//     LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");

//     if(mt2.tagCount == 0) {
//       doRejectUpdate = true;
//     }
    
//     if(!doRejectUpdate){
//       swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
//       swerve.addVisionMeasurement(
//           mt2.pose,
//           mt2.timestampSeconds);
//       limelightPublisher.set(mt2.pose);
//       updatePublisher.set(++odometryUpdates);
//     }
//     else {
//       discardPublisher.set(++odometryDiscards);
//     }

//     LLtimestamp.set(mt2.timestampSeconds);
//     fpgaTimestamp.set(Utils.getCurrentTimeSeconds());
//     LLtoFPGA.set(Utils.fpgaToCurrentTime(mt2.timestampSeconds));
//   }
// }