// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  public final ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private Command m_autonomousCommand; //AutoRoutine
  private final AutoFactory autoFactory;
  private final RobotContainer m_robotContainer;
  private final AutoChooser autoChooser;
  //public ShuffleboardTab matchTab;
  
  public Robot() {
    m_robotContainer = new RobotContainer();
    //matchTab = Shuffleboard.getTab("match");
    var drive = m_robotContainer.drivetrain;
    

    autoFactory = new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::resetPose, // A function that resets the current robot pose to the provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            drive // The drive subsystem
    );
    autoChooser = new AutoChooser();

    // Add options to the chooser
     autoChooser.addRoutine("Test Path", this::test);
     autoChooser.addRoutine("HTW", this::htw);
    //autoChooser.addCmd("Test Path", this::test);

    // Put the auto chooser on the dashboard
    //SmartDashboard.putData(autoChooser);
    matchTab.add("auto chooser lol", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

   
    //private Command exampleAutoCommand() {
        // ...
   // }

  // AUTO FACTORY METHOD
  private AutoRoutine test() {
    AutoRoutine routine = autoFactory.newRoutine("Test");
    // Load the routine's trajectories
    // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
    AutoTrajectory testPath = routine.trajectory("test");
    testPath.atPose("wait 5s", 0.2, 0.5).onTrue(new WaitCommand(2.0));
    // When the routine begins, reset odometry and start the first trajectory (1)f

    routine.active().onTrue(
        Commands.sequence(
            testPath.resetOdometry(),
            testPath.cmd()
            //testPath.atPose("wait 5s", 0.2, 0.5).onTrue(new WaitCommand(2.0)),

            //testPath.cmd()
            
        )
    ); 
    // testPath.atPose("wait 5s", 0.2, 0.5).onTrue(new WaitCommand(2.0));
    //routine.active().onTrue(Commands.print("Started the routine!"));

    return routine;
  }

 private AutoRoutine htw() {
    AutoRoutine routine = autoFactory.newRoutine("HTW");
    // Load the routine's trajectories
    // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
     AutoTrajectory hubtowershoot = routine.trajectory("H_TW");

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(
        Commands.sequence(
            hubtowershoot.resetOdometry(),
            hubtowershoot.cmd()
            
        )
    );    

    return routine;
  }

private AutoRoutine otctw() {
    AutoRoutine routine = autoFactory.newRoutine("OTCTW");
    // Load the routine's trajectories
    // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
     AutoTrajectory fuelshootclimb = routine.trajectory("OT_C_TW");

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(
        Commands.sequence(
            fuelshootclimb.resetOdometry(),
            fuelshootclimb.cmd()
            
        )
    );    

    return routine;
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