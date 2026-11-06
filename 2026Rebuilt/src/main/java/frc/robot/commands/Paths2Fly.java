// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.LinearPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Paths2Fly extends Command {
  /** Creates a new Paths2Fly. */
  // The trip planner. First pair of limits: top speed (m/s) and acceleration (m/s²) for
// driving. Second pair: the same for turning (rad/s, rad/s²).
// TODO: tune to what your drivetrain can do.
  private final LinearPath path2fly =
    new LinearPath(
        new TrapezoidProfile.Constraints(4.0, 8.0),
        new TrapezoidProfile.Constraints(Math.PI, 2.0 * Math.PI));

  private Pose2d targetPose; 
  private double heading;
  private LinearPath path; 
  private RobotContainer m_robotContainer = new RobotContainer();
  private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);
  

  // Where the robot was, and how fast it was moving, when the command started. The whole
// trip is planned from this one snapshot.
private LinearPath.State startState = new LinearPath.State();
// When the command started. (now - startTime) says how far into the trip we are.
private double startTime;

/** Takes the starting snapshot and starts the trip clock. */

  public Paths2Fly(Pose2d targetPose, double heading) {
     this.targetPose = targetPose; 
     this.heading = heading;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    startState = new LinearPath.State(m_robotContainer.swerve.getPose(), m_robotContainer.swerve.getState().Speeds);
    startTime = Utils.getCurrentTimeSeconds();
    xController.reset();
    yController.reset();
    headingController.reset();
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Ask the plan where we should be, this many seconds into the trip.
      double t = Utils.getCurrentTimeSeconds() - startTime;
      LinearPath.State setpoint = path.calculate(t, startState, targetPose);
    
      Pose2d measuredPose = m_robotContainer.swerve.getPose();
    
      // The plan's velocity does the driving. Each PID call below adds a small correction that
      // pulls the measured pose back onto the planned pose.
      ChassisSpeeds feedforward = setpoint.speeds;
      double vx = feedforward.vxMetersPerSecond + xController.calculate(measuredPose.getX(), setpoint.pose.getX());
      double vy = feedforward.vyMetersPerSecond + yController.calculate(measuredPose.getY(), setpoint.pose.getY());
      double omega =
          feedforward.omegaRadiansPerSecond
              + headingController.calculate(
                  measuredPose.getRotation().getRadians(), setpoint.pose.getRotation().getRadians());
    
      m_robotContainer.swerve.setControl(m_robotContainer.drive.withVelocityX(vx));
      m_robotContainer.swerve.setControl(m_robotContainer.drive.withVelocityY(vy));
      m_robotContainer.swerve.setControl(m_robotContainer.drive.withRotationalRate(omega));
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
      m_robotContainer.swerve.applyRequest(() -> brake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false; 
    return m_robotContainer.swerve.getState().Pose == targetPose;
  }
}
