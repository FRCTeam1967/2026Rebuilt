package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.kRobotToCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** standalone photonvision test */
public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  private Optional<PhotonPipelineResult> latestResult = Optional.empty();
  private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();
  private double lastConsolePrintTime;

  /** sets up the camera and pose estimator */
  public VisionSubsystem(String cameraName) {
    camera = new PhotonCamera(cameraName);
    AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    poseEstimator = new PhotonPoseEstimator(fieldLayout, kRobotToCamera);
    SmartDashboard.putString("Vision/Camera Name", cameraName);
  }

  /** checks the camera connection */
  public boolean isCameraConnected() {
    return camera.isConnected();
  }

  /** gets the latest result */
  public Optional<PhotonPipelineResult> getLatestResult() {
    return latestResult;
  }

  /** checks for targets */
  public boolean hasTarget() {
    return latestResult.map(PhotonPipelineResult::hasTargets).orElse(false);
  }

  /** gets the best target */
  public Optional<PhotonTrackedTarget> getBestTarget() {
    return latestResult
        .filter(PhotonPipelineResult::hasTargets)
        .map(PhotonPipelineResult::getBestTarget);
  }

  /** gets the latest robot pose estimate */
  public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
    return latestEstimatedPose;
  }

  /** gets the estimated 3d pose */
  public Optional<Pose3d> getEstimatedPose3d() {
    return latestEstimatedPose.map(estimate -> estimate.estimatedPose);
  }

  /** reads frames and updates telemetry */
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Vision/Camera Connected", isCameraConnected());
    SmartDashboard.putBoolean(
        "Vision/NetworkTables Connected", NetworkTableInstance.getDefault().isConnected());

    var results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      SmartDashboard.putBoolean("Vision/Has New Frame", false);
      return;
    }

    latestResult = Optional.of(results.get(results.size() - 1));
    var result = latestResult.orElseThrow();

    // use multi-tag first, then single-tag
    latestEstimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
    if (latestEstimatedPose.isEmpty()) {
      latestEstimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
    }

    SmartDashboard.putBoolean("Vision/Has New Frame", true);
    SmartDashboard.putNumber("Vision/Latency ms", result.metadata.getLatencyMillis());
    SmartDashboard.putNumber("Vision/Target Count", result.getTargets().size());
    SmartDashboard.putBoolean("Vision/Has Target", result.hasTargets());

    latestEstimatedPose.ifPresent(
        estimate -> {
          Pose3d pose = estimate.estimatedPose;
          SmartDashboard.putBoolean("Vision/Has Estimated Pose", true);
          SmartDashboard.putNumber("Vision/Robot X m", pose.getX());
          SmartDashboard.putNumber("Vision/Robot Y m", pose.getY());
          SmartDashboard.putNumber("Vision/Robot Z m", pose.getZ());
          SmartDashboard.putNumber(
              "Vision/Robot Heading deg", pose.getRotation().toRotation2d().getDegrees());
          SmartDashboard.putNumber("Vision/Pose Timestamp s", estimate.timestampSeconds);
        });
    if (latestEstimatedPose.isEmpty()) {
      SmartDashboard.putBoolean("Vision/Has Estimated Pose", false);
    }

    if (!result.hasTargets()) {
      return;
    }

    var target = result.getBestTarget();
    SmartDashboard.putNumber("Vision/Tag ID", target.getFiducialId());
    SmartDashboard.putNumber("Vision/Yaw deg", target.getYaw());
    SmartDashboard.putNumber("Vision/Pitch deg", target.getPitch());
    SmartDashboard.putNumber("Vision/Area percent", target.getArea());
    SmartDashboard.putNumber("Vision/Pose Ambiguity", target.getPoseAmbiguity());
    SmartDashboard.putNumber(
        "Vision/Distance m", target.getBestCameraToTarget().getTranslation().getNorm());

    double now = Timer.getFPGATimestamp();
    if (now - lastConsolePrintTime >= 1.0) {
      System.out.printf(
          "PhotonVision: tag=%d yaw=%.1f pitch=%.1f distance=%.2fm latency=%.1fms%n",
          target.getFiducialId(),
          target.getYaw(),
          target.getPitch(),
          target.getBestCameraToTarget().getTranslation().getNorm(),
          result.metadata.getLatencyMillis());
      lastConsolePrintTime = now;
    }
  }
}
