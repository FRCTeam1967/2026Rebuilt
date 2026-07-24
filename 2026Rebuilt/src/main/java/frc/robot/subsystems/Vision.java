// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// //import frc.robot.LimelightHelpers;  
// import frc.robot.Constants;
// import java.util.function.BooleanSupplier;
// import dev.doglog.DogLog;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.HttpCamera;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// public class Vision extends SubsystemBase {
//   private double maxAngularRate;
//   private Swerve swerve;
//   private Translation2d hubPose;
  
  
//   public Vision(Swerve drivetrain, double maxAngularRate) {
//     this.swerve = drivetrain;
//     this.maxAngularRate = maxAngularRate;
//   }


//   private Translation2d getHubPose() {
//     Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
//     //FOR SIMULATION:
//     //hubPose = Constants.Vision.BLUE_HUB_POSE;
//     //hubPose = Constants.Vision.RED_HUB_POSE;

//     if (alliance == Alliance.Blue) {
//       hubPose = Constants.Vision.BLUE_HUB_POSE;
//     } else {
//       hubPose = Constants.Vision.RED_HUB_POSE;
//     }

//     return hubPose;
//   }

//   public double getDisFromHub() {
//     hubPose = getHubPose();

//     Translation2d ourPose = swerve.getPose().getTranslation();

//     double eucDist = Math.hypot(ourPose.getX() - hubPose.getX(), ourPose.getY() - hubPose.getY());
//     // DogLog.log("Vision/dist from hub", eucDist);
    
//     // if (Constants.Vision.verboseLogging) {
//     //   DogLog.log("Vision/target hub", hubPose);
//     // }
    
//     return eucDist;
//   }

//   public double getAngleToHub() {
//     hubPose = getHubPose();
//     Translation2d ourPose = swerve.getPose().getTranslation();

//     double xDist = (hubPose.getX() - ourPose.getX());
//     double yDist = (hubPose.getY() - ourPose.getY());

//     //tan(angle) opposite / adjacent = ∆y/∆x so angle = arctan(∆y/∆x)
//     double angle = Math.atan2(yDist, xDist);

//     // if (Constants.Vision.verboseLogging) {
//     //   DogLog.log("Vision/raw angle to hub", angle);
//     // }

//     // if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
//     //   angle += Math.PI;
//     // }

//     // DogLog.log("Vision/angle to hub", angle);
//     return (angle);
//   }

//   public boolean isAligned() {
//     return (getAngleToHub() <= 0.174533); //5 degrees to radians
//   }

//   public BooleanSupplier getIsAligned() {
//     return (() -> getAngleToHub() <= 0.0872665); //5 degrees to radians
//   }


//   @Override
//   public void periodic() {
//     hubPose = Constants.Vision.BLUE_HUB_POSE; // In case we're not connected yet
//     if (DriverStation.getAlliance().isPresent()) {
//       if (DriverStation.getAlliance().get() == Alliance.Blue) {
//         hubPose = Constants.Vision.BLUE_HUB_POSE;
//         hubPose = Constants.Vision.BLUE_HUB_POSE;
//       } else {
//         hubPose = Constants.Vision.RED_HUB_POSE;
//         hubPose = Constants.Vision.RED_HUB_POSE;
//       }
//     }

//     this.getDisFromHub();
//   }
// }