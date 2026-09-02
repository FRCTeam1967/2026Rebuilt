package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

/** Standalone PhotonVision checkout for a roboRIO with no drivetrain or other mechanisms. */
public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera;
  private double lastConsolePrintTime;

  //connects to photon vision
  public VisionSubsystem(String cameraName) {
    camera = new PhotonCamera(cameraName);
    SmartDashboard.putString("Vision/Camera Name", cameraName);
  }

  //checks if camera is connected and displays details on smart dashboard
  @Override
  public void periodic() {
    boolean connected = camera.isConnected();
    SmartDashboard.putBoolean("Vision/Camera Connected", connected);
    SmartDashboard.putBoolean(
        "Vision/NetworkTables Connected", NetworkTableInstance.getDefault().isConnected());

    var results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      SmartDashboard.putBoolean("Vision/Has New Frame", false);
      return;
    }
    var result = results.get(results.size() - 1);
    SmartDashboard.putBoolean("Vision/Has New Frame", true);
    SmartDashboard.putNumber("Vision/Latency ms", result.metadata.getLatencyMillis());
    SmartDashboard.putNumber("Vision/Target Count", result.getTargets().size());
    SmartDashboard.putBoolean("Vision/Has Target", result.hasTargets());

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
