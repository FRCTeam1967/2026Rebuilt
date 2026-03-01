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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MovePivot;
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
    autoChooserLOL.addRoutine("Trench to Neutral Intake Climb", this::otctw);    
    
    //m_robotContainer.autoChooserLOL.addRoutine("OTN", this::otn);
    //autoChooserLOL.addRoutine("disSensorTest", this::disSensorTest);
    RobotModeTriggers.autonomous().whileTrue(autoChooserLOL.selectedCommandScheduler());
    }

    public void configDashboard(ShuffleboardTab tab) {
        tab.add("auto chooser lol", autoChooserLOL).withWidget(BuiltInWidgets.kComboBoxChooser);
        // tab.addDouble("Dis Sensor Values", () -> disSensor.getDistance().refresh().getValueAsDouble()).withWidget(BuiltInWidgets.kTextView);
    }

   
  private AutoRoutine otn() {
    AutoRoutine routine = autoFactory.newRoutine("OT_N");
    AutoTrajectory trenchNeutral = routine.trajectory("OT_N");
    //double initialOrientation = trenchNeutral.getInitialPose().get().getRotation().getDegrees();
    routine.active().onTrue(
      Commands.sequence(
        //step one: set gyro to starting heading (flips for alliance)
          //new InstantCommand(() -> m_robotContainer.drivetrain.getPigeon2().setYaw(initialOrientation)),
          trenchNeutral.resetOdometry(),
           //step three: set LL heading to gyro (aka starting) heading
         // new InstantCommand(() -> LimelightHelpers.SetRobotOrientation("limelight-front", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0)),
          trenchNeutral.cmd() 
    ));
    trenchNeutral.atPose("Deploy Intake", 0.2,0.2).onTrue(
      Commands.parallel(
        new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
        new RunIndexer(m_robotContainer.indexer, 10.0)
      )
    );

    return routine;
  }

  private AutoRoutine dtdtw(){
    AutoRoutine routine = autoFactory.newRoutine("DTW");

    AutoTrajectory trenchToDepot = routine.trajectory("DT_D");
    AutoTrajectory depotToShoot = routine.trajectory("D_Shoot");
    AutoTrajectory shootToClimb = routine.trajectory("Shoot_TW");
    double initialOrientation = trenchToDepot.getInitialPose().get().getRotation().getDegrees();

    routine.active().onTrue(
      Commands.sequence(
        //step one: set gyro to starting heading (flips for alliance)
        new InstantCommand(() -> m_robotContainer.drivetrain.getPigeon2().setYaw(initialOrientation)),
        trenchToDepot.resetOdometry(),
        //step three: set LL heading to gyro (aka starting) heading
        new InstantCommand(() -> LimelightHelpers.SetRobotOrientation("limelight-front", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0)),
              
        trenchToDepot.cmd()
      )
    );
    trenchToDepot.atPose("Deploy Intake", 0.02, Math.PI/6).onTrue(
      Commands.parallel(
        new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION),
        new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
        new RunIndexer(m_robotContainer.indexer, 10.0)
      ).withTimeout(6)
      .andThen(depotToShoot.cmd())
    );
    depotToShoot.atPose("Start Shoot", 0.02, Math.PI/6).onTrue(
      Commands.parallel(
        new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.PRELOAD_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
        Commands.sequence(
          new WaitUntilCommand(() -> m_robotContainer.flywheelShooter.reachedShooterSpeed()),
          Commands.parallel(
            new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
            new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED)
          )
        )
      ).withTimeout(5.0)
      .andThen(shootToClimb.cmd())
    );
    //TODO: add climb sequencing
    
    return routine;
  }
  private AutoRoutine htw() {
    AutoRoutine routine = autoFactory.newRoutine("HTW");
    AutoTrajectory hubTowerShoot = routine.trajectory("H_TW");
    AutoTrajectory shootFromABitBack = routine.trajectory("ShootFromABitBack");
    
    routine.active().onTrue(
      Commands.sequence(
        shootFromABitBack.resetOdometry(),
        shootFromABitBack.cmd()
      )
    ); 
    shootFromABitBack.atPose("Deploy Intake", 0.2, Math.PI/4).onTrue(
      Commands.parallel(
        new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION),
        new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
        new RunIndexer(m_robotContainer.indexer, 10.0)
      ).withTimeout(3)
    );
    shootFromABitBack.done().onTrue(
      Commands.sequence(
        new ParallelRaceGroup(
          new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
          new WaitCommand(2.5)
        ),
        new ParallelCommandGroup(
                new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
                new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED),
                new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION)
              ).withTimeout(5.0)
      )
      .andThen(hubTowerShoot.cmd())
      //TODO: add climb sequencing
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

    AutoTrajectory trenchToCenter = routine.trajectory("OT_N_EM");
    AutoTrajectory intakeMore  = routine.trajectory("OT_N_fuelBranch_EM");
    AutoTrajectory toZone = routine.trajectory("N_Trench");
    AutoTrajectory shootClimb = routine.trajectory("TrenchScoreClimb");

        // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(
        Commands.sequence(
          trenchToCenter.resetOdometry(),
          trenchToCenter.cmd()
        )
    );
    
    trenchToCenter.atPose("Deploy Intake", 0.2, Math.PI/4).onTrue(
      Commands.parallel(
        new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION),
        new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
        new RunIndexer(m_robotContainer.indexer, 10.0)
      ).withTimeout(4)
    );

    trenchToCenter.done().onTrue(intakeMore.cmd()); //TODO: make a version of this path w/o event markers, add event marker on intakeMore to intake

    // trenchToCenter.done().onTrue(
    //   Commands.parallel(
    //     Commands.parallel(
    //       new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION),
    //       new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
    //       new RunIndexer(m_robotContainer.indexer, 10.0)
    //     ), //.withTimeout(5), 
    //     intakeMore.cmd()
    //   )
    // );

    // intakeMore.atPose("Deploy Intake", 0.02, Math.PI/4).onTrue(
    //   Commands.parallel(
    //     new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION),
    //     new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
    //     new RunIndexer(m_robotContainer.indexer, 10.0)
    //   ).withTimeout(3)
    // );

    intakeMore.done().onTrue(toZone.cmd());
    // toZone.done().onTrue(
    //   Commands.parallel(
    //     Commands.sequence(
    //       new ParallelRaceGroup(
    //         new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
    //         new WaitCommand(2.5)
    //       ),
    //       new ParallelCommandGroup(
    //               new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
    //               new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED),
    //               new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION)
    //       )
    //     ), //.withTimeout(5), 
    //     shootClimb.cmd()
    //   )
    // );

    //finish the first path and get to the intaking pose. if our distance sensor detects fuel
    //the hopper is full, so we should continue with the rest of the auto and go shoot
    // Trigger atCenter = trenchToCenter.done();
    // atCenter.and(()-> disSensor.getDistance().getValueAsDouble() >= 27).onTrue(intakeMore.cmd());//if true then intake 
    // //  //write intake for fuel traj if true 
    // atCenter.and(()-> disSensor.getDistance().getValueAsDouble() < 27).onTrue(shootClimb.cmd());
    
    // shootClimb.atPose("Start Score", 0.02,Math.PI/6).onTrue(
    //   Commands.sequence(
    //     new ParallelRaceGroup(
    //       new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
    //       new WaitCommand(2.5)
    //     ),
    //     new ParallelCommandGroup(
    //             new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
    //             new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED),
    //             new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION)
    //           ).withTimeout(5.0)
    //   )
    // );
    
    // edit command sequence for climb
    // shootClimb.atPose("Climb", 0.02,Math.PI/6).onTrue(
    //   Commands.parallel(
    //     new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.PRELOAD_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
    //     Commands.sequence(
    //       new WaitUntilCommand(() -> m_robotContainer.flywheelShooter.reachedShooterSpeed()),
    //       Commands.parallel(
    //         new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
    //         new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED)
    //       )
    //     )
    //   ).withTimeout(5.0)
    // );

    return routine;
  }
  private AutoRoutine left_Otctw() {
    AutoRoutine routine = autoFactory.newRoutine("Left Branch OTCTW");
    AutoTrajectory leftTrenchToCenter = routine.trajectory("DT_N");
    AutoTrajectory fuelBranch  = routine.trajectory("DT_N_fuelBranch");
    AutoTrajectory Shoot = routine.trajectory("DT_N_score");
    AutoTrajectory Climb = routine.trajectory("DT_Climb");

        // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(
        Commands.sequence(
          leftTrenchToCenter.resetOdometry(),
          leftTrenchToCenter.cmd()
        )
    );
    
    leftTrenchToCenter.atPose("Deploy Intake", 0.02, Math.PI/6).onTrue(
      Commands.parallel(
        new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION),
        new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
        new RunIndexer(m_robotContainer.indexer, 10.0)
      ).withTimeout(3)
    );

    //finish the first path and get to the intaking pose. if our distance sensor detects fuel
    //the hopper is full, so we should continue with the rest of the auto and go shoot
    // Trigger atCenter = trenchToCenter.done();
    // atCenter.and(()-> disSensor.getDistance().getValueAsDouble() >= 27).onTrue(intakeMore.cmd());//if true then intake 
    // // fuelBranch.active().whileTrue(Commands.parallel(
        //new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION),
        //new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
        //new RunIndexer(m_robotContainer.indexer, 10.0)
      //).withTimeout(3)
      //fuelBranch.active.onTrue(Shoot.cmd)
     //write intake for fuel traj if true 
    // atCenter.and(()-> disSensor.getDistance().getValueAsDouble() < 27).onTrue(shoot.cmd());
    
    Shoot.atPose("Start Score", 0.02,Math.PI/6).onTrue(
      Commands.parallel(
        new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.PRELOAD_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
        Commands.sequence(
          new WaitUntilCommand(() -> m_robotContainer.flywheelShooter.reachedShooterSpeed()),
          Commands.parallel(
            new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
            new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED)
          )
        )
      ).withTimeout(5.0) // might have to edit timeout
    );
    
    // edit command sequence for climb
    Climb.atPose("Climb", 0.02,Math.PI/6).onTrue(
      Commands.parallel(
        new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.PRELOAD_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
        Commands.sequence(
          new WaitUntilCommand(() -> m_robotContainer.flywheelShooter.reachedShooterSpeed()),
          Commands.parallel(
            new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
            new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED)
          )
        )
      ).withTimeout(5.0)
    );

    return routine;
  }

  private AutoRoutine left_depotScore() {
    AutoRoutine routine = autoFactory.newRoutine("left_depotScore");
    AutoTrajectory depotIntake = routine.trajectory("DT_D");
    AutoTrajectory goToScore  = routine.trajectory("DT_score_intake");
    AutoTrajectory Intake = routine.trajectory("DScoreToC");
     AutoTrajectory Score = routine.trajectory("DT_N_score");

        // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(
        Commands.sequence(
          depotIntake.resetOdometry(),
          depotIntake.cmd()
        )
    );
    
    depotIntake.atPose("Deploy Intake", 0.02, Math.PI/6).onTrue(
      Commands.parallel(
        new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION),
        new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
        new RunIndexer(m_robotContainer.indexer, 10.0)
      ).withTimeout(3)
    );
    depotIntake.done().onTrue(goToScore.cmd());

    goToScore.atPose("Start Score", 0.02, Math.PI/6).onTrue(
      Commands.parallel(
        new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.PRELOAD_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
        Commands.sequence(
          new WaitUntilCommand(() -> m_robotContainer.flywheelShooter.reachedShooterSpeed()),
          Commands.parallel(
            new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
            new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED)
          )
        )
      ).withTimeout(5.0)
    );
    goToScore.done().onTrue(Intake.cmd());
    //finish the first path and get to the intaking pose. if our distance sensor detects fuel
    //the hopper is full, so we should continue with the rest of the auto and go shoot
    // Trigger atCenter = trenchToCenter.done();
    // atCenter.and(()-> disSensor.getDistance().getValueAsDouble() >= 27).onTrue(intakeMore.cmd());//if true then intake 
    // // fuelBranch.active().whileTrue(Commands.parallel(
        //new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION),
        //new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
        //new RunIndexer(m_robotContainer.indexer, 10.0)
      //).withTimeout(3)
      //fuelBranch.active.onTrue(Shoot.cmd)
     //write intake for fuel traj if true 
    // atCenter.and(()-> disSensor.getDistance().getValueAsDouble() < 27).onTrue(shoot.cmd());
    
    Intake.atPose("Deploy Intake", 0.02,Math.PI/6).onTrue(
      Commands.parallel(
        new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION),
        new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
        new RunIndexer(m_robotContainer.indexer, 10.0)
      ).withTimeout(3) // might have to edit timeout
    );
    Intake.done().onTrue(Score.cmd());
    Score.atPose("Start Score", 0.02, Math.PI/6).onTrue(
      Commands.parallel(
        new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.PRELOAD_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
        Commands.sequence(
          new WaitUntilCommand(() -> m_robotContainer.flywheelShooter.reachedShooterSpeed()),
          Commands.parallel(
            new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
            new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED)
          )
        )
      ).withTimeout(5.0)
    );
    
    return routine;
  }
  
  private AutoRoutine otc2x(){
    AutoRoutine routine = autoFactory.newRoutine("OTC 2 Cycle");
    AutoTrajectory trenchNeutral = routine.trajectory("OT_N");
    AutoTrajectory neutralScore = routine.trajectory("OT_N_score");
    AutoTrajectory scoreNeutral = routine.trajectory("ScoreToN");
    routine.active().onTrue(
      Commands.sequence(
        trenchNeutral.resetOdometry(),
        trenchNeutral.cmd()
      )
    );
    // trenchNeutral.atPose("Start Intake", 0.02, Math.PI/6).onTrue(
    //    Commands.parallel(
    //     new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION),
    //     new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
    //     new RunIndexer(m_robotContainer.indexer, 10.0)
    //   ).withTimeout(3)
    // );
    trenchNeutral.done().onTrue(
      neutralScore.cmd()
    );
    neutralScore.done().onTrue(
      Commands.parallel(
        new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.PRELOAD_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
        Commands.sequence(
          new WaitUntilCommand(() -> m_robotContainer.flywheelShooter.reachedShooterSpeed()),
          Commands.parallel(
            new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
            new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED)
          )
        )
      ).withTimeout(5.0)
      .andThen(scoreNeutral.cmd())
    );
    scoreNeutral.atPose("Start Intake", 0.02, Math.PI/6).onTrue(
      Commands.parallel(
        new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION),
        new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
        new RunIndexer(m_robotContainer.indexer, 10.0)
      ).withTimeout(3)
    );
    scoreNeutral.done().onTrue(
      neutralScore.cmd()
    );
     neutralScore.done().onTrue(
      Commands.parallel(
        new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.PRELOAD_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
        Commands.sequence(
          new WaitUntilCommand(() -> m_robotContainer.flywheelShooter.reachedShooterSpeed()),
          Commands.parallel(
            new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
            new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED)
          )
        )
      ).withTimeout(5.0));
    return routine;
    
    } 
  private AutoRoutine otoc(){
    AutoRoutine routine = autoFactory.newRoutine("OTOC");
    AutoTrajectory outpost = routine.trajectory("OT_O");
    AutoTrajectory scoreClimb = routine.trajectory("O_score_TW");
    routine.active().onTrue(
      Commands.sequence(
        outpost.resetOdometry(),
        outpost.cmd()
      )
    );
    outpost.atPose("Deploy Intake", 0.02, Math.PI/6).onTrue(
       Commands.parallel(
        new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION),
        new RunIntake(m_robotContainer.intake, Constants.Intake.INTAKE_MOTOR_SPEED), 
        new RunIndexer(m_robotContainer.indexer, 10.0)
      ).withTimeout(3)
    );
    outpost.done().onTrue(
      scoreClimb.cmd()
    );
    scoreClimb.atPose("Start Score", 0.02, Math.PI/6).onTrue(
      Commands.parallel(
        new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.PRELOAD_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
        Commands.sequence(
          new WaitUntilCommand(() -> m_robotContainer.flywheelShooter.reachedShooterSpeed()),
          Commands.parallel(
            new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
            new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED)
          )
        )
      ).withTimeout(5.0)
    );
    scoreClimb.atPose("Climb", 0.02, Math.PI/6).onTrue(
      Commands.parallel(
        new RunFlywheelShooter(m_robotContainer.flywheelShooter, Constants.FlywheelShooter.PRELOAD_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
        Commands.sequence(
          new WaitUntilCommand(() -> m_robotContainer.flywheelShooter.reachedShooterSpeed()),
          Commands.parallel(
            new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
            new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED)
          )
        )
      ).withTimeout(5.0)
    );
   //add in climb code 
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
