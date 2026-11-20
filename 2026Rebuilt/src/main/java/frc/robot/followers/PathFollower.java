package frc.robot.followers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A path follower calculates the robot speeds needed to drive from the current pose to some target
 * pose. This is distinct from a path tracker, which chooses what that target pose is.
 */
public interface PathFollower {
  /**
   * Calculate the robot speeds needed to drive the robot from the current pose to the target pose.
   *
   * @param currentSpeeds The current field relative speeds of the robot.
   * @param currentPose The current pose of the robot.
   * @param targetPose The pose the robot should drive to.
   * @param currentPoint The current point being tracked (for constraint information).
   * @param currentPointIndex The index of the current point being tracked.
   * @return The field relative chassis speeds the robot should drive at to reach the target pose.
   */
  ChassisSpeeds calculateSpeeds(
      ChassisSpeeds currentSpeeds,
      Pose2d currentPose,
      Pose2d targetPose,
      int currentPointIndex);

  /**
   * Reset the internal state of the path follower to match the current robot state. This should be
   * called when starting to follow a new path segment.
   *
   * @param currentPose The current pose of the robot.
   * @param currentSpeeds The current field relative speeds of the robot.
   */
  void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds);
}