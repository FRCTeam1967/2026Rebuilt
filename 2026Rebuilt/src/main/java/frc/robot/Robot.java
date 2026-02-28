// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;               

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.MoveClimbUp;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
//import frc.robot.LimelightHelpers.PoseEstimate;
import dev.doglog.DogLog;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pivot;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;

  boolean enableLimelight = false;

  private final StructPublisher<Pose2d> choreoPublisher;
  //private final NetworkTableListener autoPublisher;

  private Command m_autonomousCommand; //AutoRoutine
  
  
  //public final AutoChooser autoChooserLOL = new AutoChooser();
  //public ShuffleboardTab matchTab;
  
  public Robot() {
    //matchTab = Shuffleboard.getTab("match");
    //m_gerryRig = new GerryRig();
    // Put the auto chooser on the dashboard
    //matchTab.add("auto chooser lol", autoChooserLOL).withWidget(BuiltInWidgets.kComboBoxChooser);
    //SmartDashboard.putData(autoChooser)
  choreoPublisher = NetworkTableInstance.getDefault().getTable("limelight-front").getStructTopic("Limelight Pose", Pose2d.struct).publish();
  }

   
    //private Command exampleAutoCommand() {
        // ...
   // }
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

// private Command testCmd() {
//     return Commands.sequence(
//       autoFactory.resetOdometry("test"),
//       autoFactory.trajectoryCmd("test")
//     );
//   }
//gerryrig auto 
// private AutoRoutine runMotor() {
//   AutoRoutine routine = autoFactory.newRoutine("run");
//   // AutoTrajectory gerryMotor = routine.trajectory("run");
//   // Load the routine's trajectories

//   // When the routine begins, reset odometry and start the first trajectory (1)
// //   routine.active().onTrue(new RunCommand(() -> m_gerryRig.runMotor(0.7), m_gerryRig));

//   return routine;
// }

// public void addToChooser(String title, AutoRoutine routine) {
//   autoChooser.addRoutine(title, routine);
// }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }


  /*
   * Limelight IMU Modes:
   * 0: No internal IMU processing. MT2 uses interpolated yaw from robot's gyro sent via SetRobotOrientation().
   * 1: Internal IMU offset is calibrated to match external yaw each frame (seeding). MT2 still uses external yaw for botpose.
   * 2: Uses internal IMU's fused yaw only. No external input required.
   * 3: Complementary filter fuses internal IMU with MT1 vision yaw. When MT1 gets a valid pose, it slowly corrects internal IMU drift.
   * 4: Complementary filter fuses internal IMU with external yaw from SetRobotOrientation(). This is the recommended mode, as the internal IMU's 
   * 1khz update rate is utilized for frame-by-frame motion while the robot's IMU corrects for any drift over time.
   */

  @Override
  public void disabledInit() {
    SignalLogger.stop();
  }


  @Override
  public void disabledPeriodic() {
    LimelightHelpers.SetIMUMode("limelight-front", 0);
    LimelightHelpers.SetThrottle("limelight-front", 200);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    LimelightHelpers.SetIMUMode("limelight-front", 0); // robot gyro
    LimelightHelpers.SetThrottle("limelight-front", 50);

    // LimelightHelpers.SetIMUMode("limelight-back", 0); // robot gyro
    // LimelightHelpers.SetThrottle("limelight-back", 50);
    SignalLogger.start();
  }

  @Override
  public void autonomousPeriodic() {
    LimelightHelpers.SetIMUMode("limelight-front", 0); // robot gyro

    // LimelightHelpers.SetIMUMode("limelight-back", 0); // robot gyro
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    LimelightHelpers.SetThrottle("limelight-front", 0);
    // LimelightHelpers.SetThrottle("limelight-back", 0);
    m_robotContainer.visionUpdate.setFirstVisionPose();
    SignalLogger.start();
  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.flywheelShooter.logLeftShooterSpeed();
    m_robotContainer.flywheelShooter.logRightShooterSpeed();
    m_robotContainer.flywheelShooter.logAverageShooterSpeed();
    m_robotContainer.hood.logAbsEncoder();
    DogLog.log("TargetVelocity", Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED);

    LimelightHelpers.SetIMUMode("limelight-front", 0); //robot gyro
    // LimelightHelpers.SetIMUMode("limelight-back", 0); //robot gyro
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
  
  public void simulationInit() {
    m_robotContainer.pivot.simulationInit();
  }

  @Override
  public void simulationPeriodic() {
    m_robotContainer.climb.simulationPeriodic();
    m_robotContainer.pivot.simulationPeriodic();
  }
}
