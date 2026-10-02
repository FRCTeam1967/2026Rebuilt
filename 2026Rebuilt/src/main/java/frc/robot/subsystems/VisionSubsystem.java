package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** One camera source. Localization owns polling so each frame is consumed once. */
public class VisionSubsystem {
  public record Observation(EstimatedRobotPose estimate, boolean multiTag) {}
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private final String telemetryPrefix;

  public VisionSubsystem(String name, Transform3d robotToCamera, AprilTagFieldLayout layout) {
    camera = new PhotonCamera(name);
    poseEstimator = new PhotonPoseEstimator(layout, robotToCamera);
    telemetryPrefix = "Vision/" + name + "/";
  }

  public boolean isCameraConnected() {
    return camera.isConnected();
  }

  public List<Observation> drainObservations() {
    SmartDashboard.putBoolean(telemetryPrefix + "Connected", isCameraConnected());
    var observations = new ArrayList<Observation>();
    var frames = camera.getAllUnreadResults();
    SmartDashboard.putBoolean(telemetryPrefix + "Has New Frame", !frames.isEmpty());
    SmartDashboard.putBoolean(telemetryPrefix + "Has New Estimate", false);
    for (var frame : frames) {
      var estimate = poseEstimator.estimateCoprocMultiTagPose(frame);
      boolean multiTag = estimate.isPresent();
      if (estimate.isEmpty()) {
        estimate = poseEstimator.estimateLowestAmbiguityPose(frame);
      }
      SmartDashboard.putNumber(telemetryPrefix + "Target Count", frame.getTargets().size());
      SmartDashboard.putNumber(telemetryPrefix + "Latency ms", frame.metadata.getLatencyMillis());
      if (estimate.isPresent()) {
        observations.add(new Observation(estimate.get(), multiTag));
        SmartDashboard.putBoolean(telemetryPrefix + "Has New Estimate", true);
        SmartDashboard.putNumber(telemetryPrefix + "Robot X m", estimate.get().estimatedPose.getX());
        SmartDashboard.putNumber(telemetryPrefix + "Robot Y m", estimate.get().estimatedPose.getY());
      }
    }
    return observations;
  }
}
