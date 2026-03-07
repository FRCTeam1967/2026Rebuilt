// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;               

import com.ctre.phoenix6.HootAutoReplay;

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

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
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


import frc.robot.commands.MovePivot;
import frc.robot.subsystems.Pivot;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;

  private boolean enableLimelight = false;
  private Command m_autonomousCommand;
  private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
    .withTimestampReplay()
    .withJoystickReplay();

  private final StructPublisher<Pose2d> choreoPublisher;
  //private final NetworkTableListener autoPublisher;  
  
  //public final AutoChooser autoChooserLOL = new AutoChooser();
  //public ShuffleboardTab matchTab;
  
  public Robot() {
    //matchTab = Shuffleboard.getTab("match");
    //m_gerryRig = new GerryRig();
    // Put the auto chooser on the dashboard
    //matchTab.add("auto chooser lol", autoChooserLOL).withWidget(BuiltInWidgets.kComboBoxChooser);
    //SmartDashboard.putData(autoChooser)
    choreoPublisher = NetworkTableInstance.getDefault().getTable("limelight-front").getStructTopic("Limelight Pose", Pose2d.struct).publish();
    m_robotContainer = new RobotContainer();
  }

   
    //private Command exampleAutoCommand() {
        // ...
   // }
  @Override
  public void robotInit() {}

    /* log and replay timestamp and joystick data */


  @Override
  public void robotPeriodic() {
      m_timeAndJoystickReplay.update();
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

  }

  @Override
  public void disabledExit() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
      if (m_autonomousCommand != null) {
          CommandScheduler.getInstance().cancel(m_autonomousCommand);
      }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
      CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

/** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

/** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
