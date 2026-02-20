// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunFlywheelShooter;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.RunIntake;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class Autoes {
    private final AutoChooser autoChooserLOL = new AutoChooser();
    private Command m_autonomousCommand; //AutoRoutine
    private final AutoFactory autoFactory;
    //private final int disSensorID = 0;
    private final RobotContainer m_robotContainer; 
    //private final CANrange disSensor = new CANrange(disSensorID, TunerConstants.kCANBus);
    // Write these configs to the CANrange


    public Autoes(RobotContainer container){
    m_robotContainer = container;
    var drive = m_robotContainer.drivetrain;

    CANrangeConfiguration config = new CANrangeConfiguration();

    config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; // If CANrange has a signal strength of at least 2000, it is a valid measurement.
    config.ProximityParams.ProximityThreshold = 0.1; // If CANrange detects an object within 0.1 meters, it will trigger the "isDetected" signal.

    config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz; // Make the CANrange update as fast as possible at 100 Hz. This requires short-range mode.

    //disSensor.getConfigurator().apply(config);

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
    //autoChooserLOL.addRoutine("Test Conditional", this::testConditional);
    autoChooserLOL.addRoutine("Hub to Tower Preload", this::htw);
    autoChooserLOL.addRoutine("Trench to Neutral Intake", this::otn);
    
    //m_robotContainer.autoChooserLOL.addRoutine("OTN", this::otn);
    //autoChooserLOL.addRoutine("disSensorTest", this::disSensorTest);
    RobotModeTriggers.autonomous().whileTrue(autoChooserLOL.selectedCommandScheduler());
    }

    public void configDashboard(ShuffleboardTab tab) {
        tab.add("auto chooser lol", autoChooserLOL).withWidget(BuiltInWidgets.kComboBoxChooser);
        // tab.addDouble("Dis Sensor Values", () -> disSensor.getDistance().refresh().getValueAsDouble()).withWidget(BuiltInWidgets.kTextView);
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
    
    AutoTrajectory trenchNeutral = routine.trajectory("OT_N");
    routine.active().onTrue(
      Commands.sequence(
          trenchNeutral.resetOdometry(),
          trenchNeutral.cmd() 
    ));
    trenchNeutral.atPose("Start Intake", 0.2,0.2).onTrue(
      Commands.parallel(
        new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
        new RunIndexer(m_robotContainer.indexer, 10.0)
      )
    );

    return routine;
  }

  private AutoRoutine htw() {
    AutoRoutine routine = autoFactory.newRoutine("HTW");
    AutoTrajectory hubTowerShoot = routine.trajectory("H_TW");
    routine.active().onTrue(
        Commands.sequence(
            hubTowerShoot.resetOdometry(),
            hubTowerShoot.cmd()
        )
    );

    hubTowerShoot.atPose("Shoot Preload", 0, Math.PI/2)
      .onTrue(
        Commands.parallel(
              new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.PRELOAD_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
              Commands.sequence(
                new WaitUntilCommand(() -> m_robotContainer.flywheelShooter.reachedShooterSpeed()),
                Commands.parallel(
                  new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
                  new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED)
                )
              )
            ).withTimeout(7)
      );

    return routine;
  }

  /**
   * @return true if the motor is running
   */
  // public double getDisSensor() {
  //   return disSensor.getDistance().refresh().getValueAsDouble();
  // }


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
    // Trigger doneGo = goToFuel.done();
    // doneGo.and(()-> disSensor.getDistance().getValueAsDouble() >=27).onTrue(fuel.cmd());//if true then intake 
    // //  //write intake for fuel traj if true 
    // doneGo.and(()-> disSensor.getDistance().getValueAsDouble() < 27).onTrue(shootClimb.cmd());
    
        //write shoot for shootClimb
    fuel.done().onTrue(shootClimb.cmd());
    shootClimb.atPose("Marker", 0.0,0.0);//keep shooting after you reach event marker name Marker
    return routine;
  }
  private AutoRoutine otc2x(){
    AutoRoutine routine = autoFactory.newRoutine("OTC 2 Cycle");
    AutoTrajectory otc2x = routine.trajectory("OT_C_2");
    routine.active().onTrue(
      Commands.sequence(
        otc2x.resetOdometry(),
        otc2x.cmd()
      )
    );
    return routine;
    
    } 
  // private AutoRoutine disSensorTest(){
  //   AutoRoutine disSensorTest = autoFactory.newRoutine("Distance Sensor Test");

  //   while (getDisSensor() >= 0.15) {
  //     Commands.sequence(
  //       new RunCommand(()-> m_gerryRig.runMotor(0.3)).withTimeout(5.0)
  //     );
  //   }

  //   Commands.sequence(
  //       new RunCommand(()-> m_gerryRig.stopMotor())
  //   );
  //   return disSensorTest;
  // }
}
