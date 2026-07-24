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


public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  private Autoes autoes;

  //private final StructPublisher<Pose2d> choreoPublisher;
  //private final NetworkTableListener autoPublisher;
  
  // public Robot() {
  //   choreoPublisher = NetworkTableInstance.getDefault().getTable("limelight-front").getStructTopic("Limelight Pose", Pose2d.struct).publish();
   
  // }

  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();
    autoes = m_robotContainer.autoes;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
  
}
