package frc.robot.subsystems;

import static frc.robot.Constants.LocalizationConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.function.Supplier;

public class LocalizationSubsystem extends SubsystemBase {
  private final Supplier<Rotation2d> gyro;
  private final Supplier<SwerveModulePosition[]> modules;
  private final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  private final VisionSubsystem[] cameras;
  private final SwerveDrivePoseEstimator estimator;
  private final Field2d field = new Field2d();
  private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/Robot/Localization/Pose", Pose2d.struct).publish();
  private double lastAcceptedTime = Double.NEGATIVE_INFINITY;
  private double resetTime = Double.NEGATIVE_INFINITY;

  public LocalizationSubsystem(Supplier<Rotation2d> gyro,
      Supplier<SwerveModulePosition[]> modules) {
    this.gyro = gyro;
    this.modules = modules;
    estimator = new SwerveDrivePoseEstimator(kKinematics, gyro.get(), modules.get(), new Pose2d());
    cameras = Arrays.stream(VisionConstants.kCameras)
        .map(c -> new VisionSubsystem(c.name(), c.robotToCamera(), layout))
        .toArray(VisionSubsystem[]::new);
    SmartDashboard.putData("Robot/Localization/Field", field);
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    estimator.updateWithTime(now, gyro.get(), modules.get());
    var observations = new ArrayList<VisionSubsystem.Observation>();
    for (var camera : cameras) {
      observations.addAll(camera.drainObservations());
    }
    observations.sort(Comparator.comparingDouble(o -> o.estimate().timestampSeconds));
    for (var observation : observations) {
      var estimate = observation.estimate();
      var pose = estimate.estimatedPose;
      double timestamp = estimate.timestampSeconds;
      if (!Double.isFinite(timestamp) || timestamp <= resetTime
          || timestamp > now || now - timestamp > kMaxAgeSeconds
          || !Double.isFinite(pose.getX()) || !Double.isFinite(pose.getY())
          || !Double.isFinite(pose.getZ())
          || !Double.isFinite(pose.getRotation().getX())
          || !Double.isFinite(pose.getRotation().getY())
          || !Double.isFinite(pose.getRotation().getZ())
          || pose.getX() < 0 || pose.getX() > layout.getFieldLength()
          || pose.getY() < 0 || pose.getY() > layout.getFieldWidth()
          || Math.abs(pose.getZ()) > kMaxHeightMeters
          || estimate.targetsUsed.isEmpty()) {
        continue;
      }
      double distance = estimate.targetsUsed.stream()
          .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
          .average().orElse(Double.NaN);
      if (!Double.isFinite(distance) || distance > kMaxDistanceMeters) {
        continue;
      }
      if (!observation.multiTag()) {
        double ambiguity = estimate.targetsUsed.get(0).getPoseAmbiguity();
        if (!Double.isFinite(ambiguity) || ambiguity < 0 || ambiguity > kMaxAmbiguity) {
          continue;
        }
      }
      double xyStdDev = Math.max(0.10, 0.08 * distance * distance);
      if (!observation.multiTag()) {
        xyStdDev *= 2.0;
      }
      double thetaStdDev = observation.multiTag()
          ? Math.max(0.20, 0.15 * distance * distance) : 1e6;
      estimator.addVisionMeasurement(pose.toPose2d(), timestamp,
          VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
      lastAcceptedTime = now;
    }
    field.setRobotPose(getRobotPose());
    posePublisher.set(getRobotPose());
    SmartDashboard.putBoolean("Robot/Localization/HasValidPoseSensorResult",
        hasValidPoseSensorResult());
  }

  public Pose2d getRobotPose() {
    return estimator.getEstimatedPosition();
  }

  public void resetRobotPose(Pose2d pose) {
    estimator.resetPosition(gyro.get(), modules.get(), pose);
    resetTime = Timer.getFPGATimestamp();
    lastAcceptedTime = Double.NEGATIVE_INFINITY;
  }

  public boolean hasValidPoseSensorResult() {
    return Timer.getFPGATimestamp() - lastAcceptedTime <= kValidTimeoutSeconds;
  }
}
