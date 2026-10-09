package frc.robot.commands;

import static frc.robot.Constants.HubAlignmentConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Holds the robot's shooting direction toward its alliance hub, without translating.
 * Adapted from the reference drive.py alignToTargetHeading behavior.
 * Output is robot-relative ChassisSpeeds in m/s and rad/s.
 */
public class AlignToHub extends Command {
  private final Supplier<Pose2d> poseSupplier;
  private final BooleanSupplier poseValid;
  private final Consumer<ChassisSpeeds> driveOutput;
  private final Supplier<Optional<Alliance>> allianceSupplier;
  private final PIDController heading = new PIDController(kP, 0.0, 0.0);
  private final Translation2d redHub;

  public AlignToHub(Subsystem drivetrain, Supplier<Pose2d> poseSupplier,
      BooleanSupplier poseValid, Consumer<ChassisSpeeds> driveOutput) {
    this(drivetrain, poseSupplier, poseValid, driveOutput, DriverStation::getAlliance);
  }

  public AlignToHub(Subsystem drivetrain, Supplier<Pose2d> poseSupplier,
      BooleanSupplier poseValid, Consumer<ChassisSpeeds> driveOutput,
      Supplier<Optional<Alliance>> allianceSupplier) {
    this.poseSupplier = poseSupplier;
    this.poseValid = poseValid;
    this.driveOutput = driveOutput;
    this.allianceSupplier = allianceSupplier;
    var field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    redHub = new Translation2d(field.getFieldLength() - kBlueHub.getX(),
        field.getFieldWidth() - kBlueHub.getY());
    heading.enableContinuousInput(-Math.PI, Math.PI);
    heading.setTolerance(kToleranceRadians);
    addRequirements(drivetrain);
    setName("Align To Hub");
  }

  @Override
  public void initialize() {
    heading.reset();
    driveOutput.accept(new ChassisSpeeds());
    SmartDashboard.putBoolean("Robot/HubAlignment/Active", true);
    SmartDashboard.putBoolean("Robot/HubAlignment/Aligned", false);
  }

  /** Field heading for a front/offset-facing shooter, with wraparound at +/- pi. */
  public static double targetHeadingRadians(Pose2d pose, Translation2d hub) {
    return MathUtil.angleModulus(Math.atan2(hub.getY() - pose.getY(),
        hub.getX() - pose.getX()) - kShooterHeadingOffsetRadians);
  }

  @Override
  public void execute() {
    var alliance = allianceSupplier.get();
    if (alliance.isEmpty()) {
      stopFor("Waiting for alliance");
      return;
    }
    if (!poseValid.getAsBoolean()) {
      stopFor("Waiting for valid localization");
      return;
    }
    var pose = poseSupplier.get();
    if (pose == null || !Double.isFinite(pose.getX()) || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getRotation().getRadians())) {
      stopFor("Invalid robot pose");
      return;
    }
    var hub = alliance.get() == Alliance.Red ? redHub : kBlueHub;
    if (pose.getTranslation().getDistance(hub) < kMinTargetDistanceMeters) {
      stopFor("Too close to hub center to calculate heading");
      return;
    }
    double target = targetHeadingRadians(pose, hub);
    double omega = MathUtil.clamp(heading.calculate(
        MathUtil.angleModulus(pose.getRotation().getRadians()), target),
        -kMaxOmegaRadiansPerSecond, kMaxOmegaRadiansPerSecond);
    boolean aligned = heading.atSetpoint();
    driveOutput.accept(new ChassisSpeeds(0.0, 0.0, aligned ? 0.0 : omega));
    SmartDashboard.putNumber("Robot/HubAlignment/Target Heading deg", Math.toDegrees(target));
    SmartDashboard.putNumber("Robot/HubAlignment/Error deg",
        Math.toDegrees(heading.getError()));
    SmartDashboard.putBoolean("Robot/HubAlignment/Aligned", aligned);
    SmartDashboard.putString("Robot/HubAlignment/Status", aligned ? "Aligned" : "Turning");
  }

  private void stopFor(String reason) {
    heading.reset();
    driveOutput.accept(new ChassisSpeeds());
    SmartDashboard.putBoolean("Robot/HubAlignment/Aligned", false);
    SmartDashboard.putString("Robot/HubAlignment/Status", reason);
  }

  // Intentionally runs while held, correcting heading again if the robot drifts.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    stopFor("Stopped");
    SmartDashboard.putBoolean("Robot/HubAlignment/Active", false);
  }
}
