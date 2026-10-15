// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;               

import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.framework.TimedRobot;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.RunCommand;

import com.ctre.phoenix6.SignalLogger;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.NetworkTableInstance;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import org.wpilib.networktables.StructPublisher;


public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;

  boolean enableLimelight = false;

  private final StructPublisher<Pose2d> choreoPublisher;
  //private final NetworkTableListener autoPublisher;
  
  public Robot() {
    choreoPublisher = NetworkTableInstance.getDefault().getTable("limelight-front").getStructTopic("Limelight Pose", Pose2d.struct).publish();
   
  }

  public void robotInit() {
    if (Constants.Logging.enableCTRELogging) {
     SignalLogger.start();
    }

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
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

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
   //removed LL IMU Mode setting bc its also in init
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    //DogLog.log("TargetVelocity", () -> Constants.Yeeter.YEETER_SPEED);

    // LimelightHelpers.SetIMUMode("limelight-front", 0); //robot gyro
    // LimelightHelpers.SetIMUMode("limelight-back", 0);
  }

  @Override
  public void teleopExit() {}

  public void testInit() {
  }

  public void testPeriodic() {}

  public void testExit() {}
  
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
