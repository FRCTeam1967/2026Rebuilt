// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
//import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  public final ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  boolean enableLimelight = false;
  
  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;


  public Robot() {
    m_robotContainer = new RobotContainer();

    autoFactory = new AutoFactory(
            m_robotContainer.drivetrain::getPose, // A function that returns the current robot pose
            m_robotContainer.drivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
            m_robotContainer.drivetrain::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            m_robotContainer.drivetrain // The drive subsystem
    );
    autoChooser = new AutoChooser();

    autoChooser.addRoutine("Test", this::Test);

    matchTab.add("auto chooser lol", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

    //RobotModeTriggers.disabled().whileTrue(autoChooser.selectedCommandScheduler());   
  }

  //CHOREO AUTO
  private AutoRoutine Test() {
        AutoRoutine routine = autoFactory.newRoutine("Test");
        // Load the routine's trajectories
        // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
        AutoTrajectory test_path = routine.trajectory("Test");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
            Commands.sequence(
                test_path.resetOdometry(),
                test_path.cmd()          
            )
        );

        LimelightHelpers.SetRobotOrientation("limelight-front", test_path.getInitialPose().get().getRotation().getDegrees(), 0, 0, 0, 0, 0);    
        m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees());

        return routine;
    }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    /* if (enableLimelight) {

      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if (limelightMeasurement.tagCount >= 2) {
        RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        RobotContainer.drivetrain.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
      }
    } */
  }

  @Override
  public void disabledInit() {
    SignalLogger.stop();
    //m_robotContainer.drivetrain.getPigeon2().setYaw(LimelightHelpers.getBotPose3d_wpiBlue("limelight-front").getRotation().getAngle());
  }

  @Override
  public void disabledPeriodic() {
    LimelightHelpers.SetIMUMode("limelight-front", 1);
    LimelightHelpers.SetThrottle("limelight-front", 200);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    LimelightHelpers.SetThrottle("limelight-front", 0);
  }

  @Override
  public void autonomousPeriodic() {
    LimelightHelpers.SetIMUMode("limelight-front", 2);
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    LimelightHelpers.SetThrottle("limelight-front", 0);
    m_robotContainer.vision.setFirstVisionPose();
  }

  @Override
  public void teleopPeriodic() {
    LimelightHelpers.SetIMUMode("limelight-front", 2); //1
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}