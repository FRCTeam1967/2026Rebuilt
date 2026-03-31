// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;               

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.networktables.StructPublisher;


public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  private Autoes autoes;

  boolean enableLimelight = false;

  private final StructPublisher<Pose2d> choreoPublisher;
  //private final NetworkTableListener autoPublisher;
  
  public Robot() {
    choreoPublisher = NetworkTableInstance.getDefault().getTable("limelight-front").getStructTopic("Limelight Pose", Pose2d.struct).publish();
   
  }

  @Override
  public void robotInit() {
    DogLog.setEnabled(Constants.Logging.enabled);
    DogLogOptions options = new DogLogOptions()
      .withCaptureConsole(Constants.Logging.captureConsole)
      .withCaptureDs(Constants.Logging.captureDS)
      .withCaptureNt(Constants.Logging.captureNT)
      .withLogExtras(Constants.Logging.enableExtras);
    DogLog.setOptions(options);
    
    if (Constants.Logging.capturePDH) {
      DogLog.setPdh(new PowerDistribution());
    }

    // Start the CTRE signal logger
    if (Constants.Logging.enableCTRELogging) {
      SignalLogger.start();
    }

    m_robotContainer = new RobotContainer();
     autoes = m_robotContainer.autoes;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    DogLog.log("dis sensor", autoes.getDisSensor());
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
    //TODO: check if this is ok; limelight stuff used to be in periodic but I moved it here
    LimelightHelpers.SetIMUMode("limelight-front", 0);
    LimelightHelpers.SetThrottle("limelight-front", 200);
    LimelightHelpers.SetIMUMode("limelight-back", 0);
    LimelightHelpers.SetThrottle("limelight-back", 200);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    LimelightHelpers.SetIMUMode("limelight-front", 0); // robot gyro
    LimelightHelpers.SetThrottle("limelight-front", 0); //used to be 50
    LimelightHelpers.SetIMUMode("limelight-back", 0);
    LimelightHelpers.SetThrottle("limelight-back", 0); //used to be 50
  }

  @Override
  public void autonomousPeriodic() {
   //removed LL IMU Mode setting bc its also in init
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    LimelightHelpers.SetThrottle("limelight-front", 0);
    LimelightHelpers.SetThrottle("limelight-back", 0);
    m_robotContainer.visabelleUpdate.setFirstVisionPose();
  }

  @Override
  public void teleopPeriodic() {
    //DogLog.log("TargetVelocity", () -> Constants.Yeeter.YEETER_SPEED);

    // LimelightHelpers.SetIMUMode("limelight-front", 0); //robot gyro
    // LimelightHelpers.SetIMUMode("limelight-back", 0);
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
