package frc.robot.subsystems;

import static frc.robot.Constants.LocalizationConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** One camera source. Localization owns polling so each frame is consumed once. */
public class VisionSubsystem {
  public record Observation(EstimatedRobotPose estimate, boolean multiTag) {}
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private final AprilTagFieldLayout layout;
  private final String telemetryPrefix;

  public VisionSubsystem(String name, Transform3d robotToCamera, AprilTagFieldLayout layout) {
    camera = new PhotonCamera(name);
    poseEstimator = new PhotonPoseEstimator(layout, robotToCamera);
    this.layout = layout;
    telemetryPrefix = "Vision/" + name + "/";
  }

  public boolean isCameraConnected() {
    return camera.isConnected();
  }

  public List<Observation> drainObservations(double now, double resetTime,
      ChassisSpeeds measuredSpeeds) {
    SmartDashboard.putBoolean(telemetryPrefix + "Connected", isCameraConnected());
    var observations = new ArrayList<Observation>();
    var frames = camera.getAllUnreadResults();
    SmartDashboard.putBoolean(telemetryPrefix + "Has New Frame", !frames.isEmpty());
    SmartDashboard.putBoolean(telemetryPrefix + "Has New Estimate", false);
    // Conservative current-speed gate, not an estimate of speed at image capture time.
    boolean motionValid = measuredSpeeds != null
        && Double.isFinite(measuredSpeeds.vxMetersPerSecond)
        && Double.isFinite(measuredSpeeds.vyMetersPerSecond)
        && Double.isFinite(measuredSpeeds.omegaRadiansPerSecond)
        && Math.hypot(measuredSpeeds.vxMetersPerSecond, measuredSpeeds.vyMetersPerSecond)
            <= kMaxVisionSpeedMetersPerSecond
        && Math.abs(measuredSpeeds.omegaRadiansPerSecond) <= kMaxVisionOmegaRadiansPerSecond;
    SmartDashboard.putBoolean(telemetryPrefix + "Motion Valid", motionValid);
    // This reuses the local reference, not the objects returned by PhotonLib.
    // Java garbage collection manages those objects; observations retain their own estimates.
    Optional<EstimatedRobotPose> estimate;
    for (var frame : frames) {
      double timestamp = frame.getTimestampSeconds();
      var targets = frame.getTargets();
      SmartDashboard.putNumber(telemetryPrefix + "Target Count", targets.size());
      SmartDashboard.putNumber(telemetryPrefix + "Latency ms", frame.metadata.getLatencyMillis());
      if (!motionValid || !Double.isFinite(timestamp) || timestamp <= resetTime
          || timestamp > now || now - timestamp > kMaxAgeSeconds) {
        continue;
      }
      estimate = Optional.empty();
      boolean multiTag = false;
      var multi = frame.getMultiTagResult();
      if (multi.isPresent()) {
        var solution = multi.get();
        double error = solution.estimatedPose.bestReprojErr;
        double ambiguity = solution.estimatedPose.ambiguity;
        var usedTargets = new ArrayList<PhotonTrackedTarget>();
        boolean allUsedTargetsValid = solution.fiducialIDsUsed.size() >= 2;
        for (short id : solution.fiducialIDsUsed) {
          var target = targets.stream().filter(t -> t.getFiducialId() == id).findFirst();
          if (target.isEmpty() || !hasValidGeometry(target.get())) {
            allUsedTargetsValid = false;
            break;
          }
          usedTargets.add(target.get());
        }
        if (allUsedTargetsValid && Double.isFinite(error) && error >= 0
            && error <= kMaxMultiTagReprojectionErrorPixels
            && Double.isFinite(ambiguity) && ambiguity >= 0 && ambiguity <= kMaxAmbiguity) {
          // The coprocessor has ALREADY solved this pose. Keep every contributor or reject it.
          // Restrict metadata to the actual contributors so distance weighting is accurate.
          estimate = poseEstimator.estimateCoprocMultiTagPose(
              new PhotonPipelineResult(frame.metadata, usedTargets, multi));
          multiTag = estimate.isPresent();
        }
      }
      if (estimate.isEmpty()) {
        // Unlike the precomputed multi-tag solution, single-tag selection can be filtered here.
        // Enable single-target pose estimation in PhotonVision for this fallback.
        var goodTargets = targets.stream().filter(this::hasValidGeometry)
            .filter(t -> Double.isFinite(t.getPoseAmbiguity())
                && t.getPoseAmbiguity() >= 0 && t.getPoseAmbiguity() <= kMaxAmbiguity)
            .toList();
        if (!goodTargets.isEmpty()) {
          estimate = poseEstimator.estimateLowestAmbiguityPose(
              new PhotonPipelineResult(frame.metadata, goodTargets, Optional.empty()));
        }
      }
      if (estimate.isPresent()) {
        observations.add(new Observation(estimate.get(), multiTag));
        SmartDashboard.putBoolean(telemetryPrefix + "Has New Estimate", true);
        SmartDashboard.putNumber(telemetryPrefix + "Robot X m", estimate.get().estimatedPose.getX());
        SmartDashboard.putNumber(telemetryPrefix + "Robot Y m", estimate.get().estimatedPose.getY());
      }
    }
    return observations;
  }

  private boolean hasValidGeometry(PhotonTrackedTarget target) {
    if (layout.getTagPose(target.getFiducialId()).isEmpty()) {
      return false;
    }
    var transform = target.getBestCameraToTarget();
    double distance = transform.getTranslation().getNorm();
    return Double.isFinite(distance) && distance > 0 && distance <= kMaxDistanceMeters
        && Double.isFinite(transform.getRotation().getX())
        && Double.isFinite(transform.getRotation().getY())
        && Double.isFinite(transform.getRotation().getZ());
  }
}
