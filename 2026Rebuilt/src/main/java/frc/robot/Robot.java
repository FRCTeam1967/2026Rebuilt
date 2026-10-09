// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;               

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.networktables.StructPublisher;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public final class Robot {
  private RobotContainer m_robotContainer;
  private Autoes autoes;

  boolean enableLimelight = false;

  private final StructPublisher<Pose2d> choreoPublisher;
  //private final NetworkTableListener autoPublisher;

  public Robot() {
    choreoPublisher = NetworkTableInstance.getDefault().getTable("limelight-front").getStructTopic("Limelight Pose", Pose2d.struct).publish();
   
  }

  
  public void robotInit() {
    DogLog.setEnabled(Constants.Logging.enabled);
    DogLogOptions options = new DogLogOptions()
      .withCaptureConsole(Constants.Logging.captureConsole)
      .withCaptureDs(Constants.Logging.captureDS)
      .withCaptureNt(Constants.Logging.captureNT)
      .withNtPublish(false)
      .withLogExtras(Constants.Logging.enableExtras);
    DogLog.setOptions(options);
    
    if (Constants.Logging.capturePDH) {
      DogLog.setPdh(new PowerDistribution());
    }

    m_robotContainer = new RobotContainer();
    autoes = m_robotContainer.autoes;
  }


  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    //DogLog.log("yeeter Speed1", m_robotContainer.yeeter.getMotorVelocity());
    //DogLog.log("dis sensor", autoes.getDisSensor());
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


  public void disabledInit() {
    //TODO: check if this is ok; limelight stuff used to be in periodic but I moved it here
    LimelightHelpers.SetIMUMode("limelight-front", 0);
    LimelightHelpers.SetThrottle("limelight-front", 200);
    LimelightHelpers.SetIMUMode("limelight-back", 0);
    LimelightHelpers.SetThrottle("limelight-back", 200);
  }

 
  public void disabledPeriodic() {
  }

 
  public void disabledExit() {}

  
  public void autonomousInit() {
    LimelightHelpers.SetIMUMode("limelight-front", 0); // robot gyro
    LimelightHelpers.SetThrottle("limelight-front", 0); //used to be 50
    LimelightHelpers.SetIMUMode("limelight-back", 0);
    LimelightHelpers.SetThrottle("limelight-back", 0); //used to be 50
  }

 
  public void autonomousPeriodic() {
   //removed LL IMU Mode setting bc its also in init
  }

 
  public void autonomousExit() {}

  
  public void teleopInit() {
    LimelightHelpers.SetThrottle("limelight-front", 0);
    LimelightHelpers.SetThrottle("limelight-back", 0);
    m_robotContainer.visabelleUpdate.setFirstVisionPose();

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

        // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        //setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }
     Logger.start();
  }

 
  public void teleopPeriodic() {
    //DogLog.log("TargetVelocity", () -> Constants.Yeeter.YEETER_SPEED);

    // LimelightHelpers.SetIMUMode("limelight-front", 0); //robot gyro
    // LimelightHelpers.SetIMUMode("limelight-back", 0);
  }

  
  public void teleopExit() {}


  public void testInit() {}


  public void testPeriodic() {}


  public void testExit() {}
  
  public void simulationInit() {
    m_robotContainer.pivot.simulationInit();
  }

  public void simulationPeriodic() {
    m_robotContainer.climb.simulationPeriodic();
    m_robotContainer.pivot.simulationPeriodic();
  }

  }


