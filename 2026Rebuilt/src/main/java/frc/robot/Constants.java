// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class VisionConstants {
    //fill mount measurements
    public record CameraConfig(String name, Transform3d robotToCamera) {}

    public static final CameraConfig[] kCameras = {
      camera("FrontLeft", 0.30, 0.30, 45),
      camera("FrontRight", 0.30, -0.30, -45),
      camera("RearLeft", -0.30, 0.30, 135),
      camera("RearRight", -0.30, -0.30, -135)
    };

    private static CameraConfig camera(String name, double x, double y, double yawDegrees) {
      return new CameraConfig(name, new Transform3d(
          new Translation3d(x, y, 0.45),
          new Rotation3d(0, 0, Math.toRadians(yawDegrees))));
    }

    private VisionConstants() {}
  }

  public static final class LocalizationConstants {
    //fill in actual module locations
    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
        new Translation2d(0.30, 0.30), new Translation2d(0.30, -0.30),
        new Translation2d(-0.30, 0.30), new Translation2d(-0.30, -0.30));
    // initial tuning values
    public static final double kMaxDistanceMeters = 5.0;
    public static final double kMaxAmbiguity = 0.20;
    public static final double kMaxMultiTagReprojectionErrorPixels = 2.0;
    public static final double kMaxVisionSpeedMetersPerSecond = 4.0;
    public static final double kMaxVisionOmegaRadiansPerSecond = 3.0;
    public static final double kMaxHeightMeters = 0.50;
    public static final double kMaxAgeSeconds = 0.50;
    public static final double kValidTimeoutSeconds = 0.50;
    private LocalizationConstants() {}
  }

  public static final class HubAlignmentConstants {
    public static final Translation2d kBlueHub = new Translation2d(4.625, 4.030);
    public static final double kP = 4.0;
    public static final double kMaxOmegaRadiansPerSecond = 2.0;
    public static final double kToleranceRadians = Math.toRadians(2.0);
    public static final double kShooterHeadingOffsetRadians = 0.0;
    public static final double kMinTargetDistanceMeters = 0.10;
    private HubAlignmentConstants() {}
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
