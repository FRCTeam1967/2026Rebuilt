// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;               

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.GerryRig;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand; //AutoRoutine
  public static RobotContainer m_robotContainer;
  
  //public final AutoChooser autoChooserLOL = new AutoChooser();
  //public ShuffleboardTab matchTab;
  
  public Robot() {
    //matchTab = Shuffleboard.getTab("match");
    //m_gerryRig = new GerryRig();
    // Put the auto chooser on the dashboard
    //matchTab.add("auto chooser lol", autoChooserLOL).withWidget(BuiltInWidgets.kComboBoxChooser);
    //SmartDashboard.putData(autoChooser)
  }

   
    //private Command exampleAutoCommand() {
        // ...
   // }
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // //m_autonomousCommand = pickupAndScoreAuto();
    // // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // m_autonomousCommand = autoChooser.selectedCommand();
  
    // if (m_autonomousCommand != null) {
    //   //autoChooser.selectedCommandScheduler();
    //   m_autonomousCommand.schedule();
    // }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
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

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}