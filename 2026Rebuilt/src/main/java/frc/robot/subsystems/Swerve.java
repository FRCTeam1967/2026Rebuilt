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


public class Swerve extends SubsystemBase {
  private final SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(
      getKinematics(), getHeading(), getModulePositions(), new Pose2d());
  private double lastResetTimestamp = Double.NEGATIVE_INFINITY;

 
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

  public Rotation2d getHeading() {
    return new Rotation2d();
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(), new SwerveModulePosition(),
      new SwerveModulePosition(), new SwerveModulePosition()
    };
  }

  public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds();
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SmartDashboard.putNumber("Robot/HubAlignment/Requested Omega rad per sec",
        speeds.omegaRadiansPerSecond);
  }
}
