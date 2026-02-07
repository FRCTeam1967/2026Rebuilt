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
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import frc.robot.subsystems.GerryRig;

public class Robot extends TimedRobot {
  public static LED ledSubsystem = new LED();
  public final ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private Command m_autonomousCommand; //AutoRoutine
  private final AutoFactory autoFactory;
  private final RobotContainer m_robotContainer;
  private final AutoChooser autoChooser;
  private Rev2mDistanceSensor distOnboard; 
  private Rev2mDistanceSensor distMXP;
  private final GerryRig m_gerryRig; 
  LEDPattern solidBlue = LEDPattern.solid(Color.kWhite);
  LEDPattern blinking = solidBlue.blink(Seconds.of(0.5)).atBrightness(Percent.of(10));
  Command blinkCommand = ledSubsystem.runPattern(blinking).ignoringDisable(true);

  //public ShuffleboardTab matchTab;
  
  public Robot() {
    m_robotContainer = new RobotContainer();
    //matchTab = Shuffleboard.getTab("match");
    var drive = m_robotContainer.drivetrain;
    m_gerryRig = new GerryRig();

    
    distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
    distMXP = new Rev2mDistanceSensor(Port.kMXP);
    
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
     autoChooser.addRoutine("Test Conditional", this::testConditional);
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
    
    AutoTrajectory testPath = routine.trajectory("test");
    // When the routine begins, reset odometry and start the first trajectory (1)f
    //testPath.atPose("wait 5s", 0.0, 0.0);
    // testPath.atTime("wait 5s").onTrue(new WaitCommand(2.0));
    routine.active().onTrue(
      Commands.sequence(
          testPath.resetOdometry(),
          testPath.cmd() 
    ));
    
    testPath.atPose("wait 5s", 0.1, 0.5).onTrue(blinkCommand.withTimeout(2.0));

    // while (m_gerryRig.getGerryVel() > 0.6) {
    //   ledSubsystem.runPattern(blinking);
    //   //intake
    //   //pid to pose forward to the gamepiece pose
    //   }
    return routine;
  }

  private AutoRoutine testConditional() {
    AutoRoutine conditional = autoFactory.newRoutine("conditionalTest");
    
    
    conditional.active().onTrue(
      new ConditionalCommand(ledSubsystem.runPattern(blinking),
        Commands.sequence(
          new RunCommand(() -> m_gerryRig.runMotor(0.3), m_gerryRig).withTimeout(3),
          new RunCommand(() -> m_gerryRig.runMotor(0.7), m_gerryRig).withTimeout(3),
          new RunCommand(() -> m_gerryRig.stopMotor(), m_gerryRig)
          ), () -> m_gerryRig.getGerryVel() > 0.5)
    );   
    return conditional;
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
    AutoTrajectory goToFuel = routine.trajectory("OT_N_TW1");
    AutoTrajectory fuel  = routine.trajectory("OT_N_fuelBranch2");
    AutoTrajectory shootClimb = routine.trajectory("OT_N_TW3");

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(
        Commands.sequence(
            goToFuel.resetOdometry(),
            goToFuel.cmd()
            
        )
    );    
    //finish the first path and get to the intaking pose. if our distance sensor detects fuel
    //the hopper is full, so we should continue with the rest of the auto and go shoot
    Trigger doneGo = goToFuel.done();
    doneGo.and(()-> distMXP.getRange()>=27).onTrue(fuel.cmd());//if true then intake 
     //write intake for fuel traj if true 
    doneGo.and(()-> distMXP.getRange()< 27).onTrue(shootClimb.cmd());
   
    //write shoot for shootClimb
    fuel.done().onTrue(shootClimb.cmd());
    shootClimb.atPose("Marker", 0.0,0.0);//keep shooting after you reach event marker name Marker
    //create an event marker with offset in choreo interface that says stop motor and then start climb in code
    return routine;
}

// private AutoRoutine gRig() {
//     AutoRoutine routine = autoFactory.newRoutine("GerryRig");
//     // Load the routine's trajectories
//     // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
//     // AutoTrajectory traj1 = routine.trajectory("gerryRig1");
//     // AutoTrajectory traj2 = routine.trajectory("gerryRig2");
//     // When the routine begins, reset odometry and start the first trajectory (1)
    
//     routine.active().onTrue(
//         Commands.sequence(
//             gerryRig1.resetOdometry(),
//             gerryRig2.cmd()
//         )
//     );    
//     gerryRig1.active().onTrue(new RunCommand(() -> m_gerryRig.runMotor(0.7), m_gerryRig));
//     Trigger atGerryRig1 = gerryRig1.done();
//     atGerryRig1.and(m_gerryRig.getGerryVel() > 0).onTrue(gerryRig2.cmd());
    
//     return routine;
// }
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