package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LocalizationConstants;

/** Placeholder matching the team's drivetrain-facing methods.
 * Replace with this chassis's generated swerve when available. No CTRE configuration,
 * motor IDs, encoder offsets, or hardware from the competition robot are included.
 * This temporary estimator uses stationary dummy odometry; it is not a drive simulation.
 */
public class Swerve extends SubsystemBase {
  private final SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(
      getKinematics(), getHeading(), getModulePositions(), new Pose2d());
  private double lastResetTimestamp = Double.NEGATIVE_INFINITY;

  // Called by localization immediately before draining cameras, so odometry precedes vision.
  // Remove this call when generated swerve owns its own odometry update thread.
  public void updatePlaceholderOdometry(double timestamp) {
    estimator.updateWithTime(timestamp, getHeading(), getModulePositions());
  }

  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    estimator.resetPosition(getHeading(), getModulePositions(), pose);
    lastResetTimestamp = Timer.getFPGATimestamp();
  }

  public double getLastResetTimestamp() {
    return lastResetTimestamp;
  }

  /** FPGA seconds. This WPILib placeholder needs no CTRE timestamp conversion. */
  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    if (timestamp > lastResetTimestamp) {
      estimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }
  }

  public SwerveDriveKinematics getKinematics() {
    return LocalizationConstants.kKinematics;
  }

  /** Real adapter must return CCW-positive heading, not a negated field-relative drive offset. */
  public Rotation2d getHeading() {
    return new Rotation2d();
  }

  /** Measured distances in meters, in the same FL, FR, RL, RR order as the kinematics. */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(), new SwerveModulePosition(),
      new SwerveModulePosition(), new SwerveModulePosition()
    };
  }

  /** Measured robot-relative motion, NOT the last commanded velocity. */
  public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds();
  }

  /** Real adapter must drive robot-relative m/s and CCW-positive rad/s without another flip. */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    SmartDashboard.putNumber("Robot/HubAlignment/Requested Omega rad per sec",
        speeds.omegaRadiansPerSecond);
  }
}
