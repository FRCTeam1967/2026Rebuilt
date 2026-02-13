// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.LEDPattern;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.GerryRig;


/** Add your docs here. */
public class Autoes {
    //private final AutoChooser autoChooserLOL = new AutoChooser();
    private Command m_autonomousCommand; //AutoRoutine
    private final AutoFactory autoFactory;
    private final int disSensorID = 0;
    private final GerryRig m_gerryRig = new GerryRig(); 
    private final RobotContainer m_robotContainer; 
    private final CANrange disSensor = new CANrange(disSensorID, TunerConstants.kCANBus);
    // Write these configs to the CANrange


    public Autoes(){
    m_robotContainer = Robot.m_robotContainer;
    var drive = m_robotContainer.drivetrain;

    CANrangeConfiguration configs = new CANrangeConfiguration(); 
    disSensor.getConfigurator().apply(configs);

    autoFactory = new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::resetPose, // A function that resets the current robot pose to the provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            drive
        );
    //autoChooser = new AutoChooser();
    // Add options to the chooser
    
    //m_robotContainer.autoChooserLOL.addRoutine("Test Path", this::test);
    //autoChooser.addRoutine("Test Conditional", this::testConditional);
    //m_robotContainer.autoChooserLOL.addRoutine("HTW", this::htw);
    //m_robotContainer.autoChooserLOL.addRoutine("OTN", this::otn);
    //m_robotContainer.autoChooserLOL.addRoutine("disSensorTest", this::disSensorTest);
    RobotModeTriggers.autonomous().whileTrue(m_robotContainer.autoChooserLOL.selectedCommandScheduler());
    }

    public void configDashboard(ShuffleboardTab tab) {
        tab.add("auto chooser lol", m_robotContainer.autoChooserLOL).withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    private AutoRoutine test() {
    AutoRoutine routine = autoFactory.newRoutine("Test");
    
    AutoTrajectory testPath = routine.trajectory("test");
    routine.active().onTrue(
      Commands.sequence(
          testPath.resetOdometry(),
          testPath.cmd() 
    ));
    return routine;
  }

  private AutoRoutine otn() {
    AutoRoutine routine = autoFactory.newRoutine("OT_N");
    
    AutoTrajectory testPath = routine.trajectory("OT_N");
    routine.active().onTrue(
      Commands.sequence(
          testPath.resetOdometry(),
          testPath.cmd() 
    ));

    return routine;
  }

  private AutoRoutine htw() {
    AutoRoutine routine = autoFactory.newRoutine("HTW");
    AutoTrajectory hubtowershoot = routine.trajectory("H_TW");
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
    AutoTrajectory goToFuel = routine.trajectory("OT_N");
    AutoTrajectory fuel  = routine.trajectory("OT_N_fuelBranch");
    AutoTrajectory shootClimb = routine.trajectory("N_ScoreClimb");

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
    doneGo.and(()-> disSensor.getDistance().getValueAsDouble() >=27).onTrue(fuel.cmd());//if true then intake 
    //  //write intake for fuel traj if true 
    doneGo.and(()-> disSensor.getDistance().getValueAsDouble() < 27).onTrue(shootClimb.cmd());
    
        //write shoot for shootClimb
    fuel.done().onTrue(shootClimb.cmd());
    shootClimb.atPose("Marker", 0.0,0.0);//keep shooting after you reach event marker name Marker
    return routine;
  }
  private AutoRoutine disSensorTest(){
    AutoRoutine disSensorTest = autoFactory.newRoutine("Distance Sensor Test");
    if (disSensor.getDistance().getValueAsDouble() <= 6){
       m_gerryRig.runMotor(0.5);
    }
    return disSensorTest;
  }
}
