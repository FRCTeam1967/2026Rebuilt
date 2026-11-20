// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

 package frc.robot.commands;



import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Paths2Fly implements PathFollower {
  private final PIDController translationController;
  private final PIDController rotationController;
  private final ConstraintsCalculator velocityConstrainer;

  public Paths2Fly(PIDController translationController, PIDController rotationController) {
    this.translationController = translationController;
    this.rotationController = rotationController;
    this.velocityConstrainer = new ConstraintsCalculator(rotationController);

    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public ChassisSpeeds calculateSpeeds(
      ChassisSpeeds currentSpeeds,
      Pose2d currentPose,
      Pose2d targetPose,
      int currentPointIndex) {
    // Get constraints for the current point
    var linearConstraints =
        segment.getLinearConstraints(currentPoint).orElseGet(LinearConstraintOptions::new);
    var angularConstraints =
        segment.getAngularConstraints(currentPoint).orElseGet(AngularConstraintOptions::new);

    var distanceToEnd = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    // Sum the total linear distance and total angular work remaining through all waypoints in the
    // segment. These are used together to estimate how long the robot has to complete its rotation,
    // which drives the dynamic angular velocity cap.
    //
    // For arc segments with a midpoint rotation, we compute angular work through the midpoint
    // (current -> midpoint + midpoint -> final) instead of using shortest-path angleModulus
    // directly to the final rotation. This ensures the velocity cap accounts for the full
    // intended rotation path and prevents the robot from trying to take a shortcut.
    var totalAngularWorkRadians = 0.0;
    var currentPointFinalRotation = currentPoint.getPose().getRotation();

    if (currentPoint.arcMidpoint().isPresent()) {
      var midRotation = currentPoint.arcMidpoint().orElseThrow().getRotation();
      // Leg 1: current rotation -> midpoint rotation
      totalAngularWorkRadians +=
          Math.abs(
              MathUtil.angleModulus(
                  midRotation.getRadians() - currentPose.getRotation().getRadians()));
      // Leg 2: midpoint rotation -> current waypoint's final rotation
      totalAngularWorkRadians +=
          Math.abs(
              MathUtil.angleModulus(
                  currentPointFinalRotation.getRadians() - midRotation.getRadians()));
    } else {
      totalAngularWorkRadians +=
          Math.abs(
              MathUtil.angleModulus(
                  targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()));
    }

    // Find total distance and angular work to end of segment
    for (int i = currentPointIndex; i < segment.points.size() - 1; i++) {
      var thisPoint = segment.points.get(i).poseSupplier().get().getPose();
      var nextPoint = segment.points.get(i + 1).getPose();

      distanceToEnd += thisPoint.getTranslation().getDistance(nextPoint.getTranslation());
      totalAngularWorkRadians +=
          Math.abs(
              MathUtil.angleModulus(
                  nextPoint.getRotation().getRadians() - thisPoint.getRotation().getRadians()));
    }

    // Calculate velocities with PID controller
    var linearVelocity = Math.abs(translationController.calculate(distanceToEnd, 0));
    var angularVelocity =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    DogLog.log("Trailblazer/Follower/AngularVelocity", angularVelocity);

    // Constrain velocities
    linearVelocity =
        velocityConstrainer.constrainLinearVelocity(
            linearVelocity, currentSpeeds, distanceToEnd, linearConstraints);

    angularVelocity =
        velocityConstrainer.constrainAngularVelocity(
            angularVelocity,
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians(),
            currentSpeeds,
            angularConstraints);

    DogLog.log("Trailblazer/Follower/AngularLimit", angularConstraints.maxVelocity());
    DogLog.log("Trailblazer/Follower/AngularACCLimit", angularConstraints.maxAcceleration());

    DogLog.log("Trailblazer/Follower/AngularVelocityAfterConstraint", angularVelocity);

    var driveDirection = MathHelpers.getDriveDirection(currentPose, targetPose);

    return new PolarChassisSpeeds(linearVelocity, driveDirection, angularVelocity);
  }

  @Override
  public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    velocityConstrainer.reset(currentSpeeds, currentPose.getRotation().getRadians());
  }
}