// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimHub;
import frc.robot.commands.AlignTowerPose;
import frc.robot.commands.MoveClimbHalfwayDown;
import frc.robot.commands.MoveClimbUp;
import frc.robot.commands.MoveClimbtoZero;
import frc.robot.commands.MovePivot;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunYeeter;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.RunEater;

/** Add your docs here. */
public class Autoes {
  private final AutoChooser autoChooserLOL = new AutoChooser();
  private final AutoFactory autoFactory;
  private final int disSensorID = 24;
  private final RobotContainer m_robotContainer; 
  //private final CANrange disSensor = new CANrange(disSensorID);
  // Write these configs to the CANrange

  public Autoes(RobotContainer container) {
    m_robotContainer = container;
    var drive = m_robotContainer.swerve;

    //CANrangeConfiguration config = new CANrangeConfiguration();

    // config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; // If CANrange has a signal strength of at least 2000, it is a valid measurement.
    // config.ProximityParams.ProximityThreshold = 0.1; // If CANrange detects an object within 0.1 meters, it will trigger the "isDetected" signal.

    // config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz; // Make the CANrange update as fast as possible at 100 Hz. This requires short-range mode.

    // disSensor.getConfigurator().apply(config);

    autoFactory = new AutoFactory(
      drive::getPose, // A function that returns the current robot pose
      drive::resetPose, // A function that resets the current robot pose to the provided Pose2d
      drive::followTrajectory, // The drive subsystem trajectory follower
      true, // If alliance flipping should be enabled
      drive
    );

    autoChooserLOL.addRoutine("OT to Outpost", this::oto);
    autoChooserLOL.addRoutine("OT Neutral 2 Cycle", this::otn2x);
    autoChooserLOL.addRoutine("DT Neutral 2 Cycle", this::dtn2x);
    autoChooserLOL.addRoutine("Hub Preload", this::hubScore);
    autoChooserLOL.addRoutine("HubToOutpost Shoot", this::hTo);
    autoChooserLOL.addRoutine("HubToDepot Shoot", this::hTd);
    autoChooserLOL.addRoutine("HubToOutpost to Neutral Intake Shoot", this::hotn);
    autoChooserLOL.addRoutine("HubToDepot to Neutral Intake Shoot", this::hdtn);
    autoChooserLOL.addRoutine("DT Neutral Bump 2 Cycle", this::dtn2xBump);
    autoChooserLOL.addRoutine("OT Neutral Bump 2 Cycle", this::otn2xBump);
    autoChooserLOL.addRoutine("DT Disrupt", this::dtndisrupt);
    autoChooserLOL.addRoutine("Hub Preload Climb", this::htw);
    autoChooserLOL.addRoutine("DELAY DT Neutral Bump 2x", this::dtn2xBumpDelay);
    autoChooserLOL.addRoutine("OT Disrupt", this::otdisrupt);
    autoChooserLOL.addRoutine("Tower Test", this::towerTest);
    autoChooserLOL.addRoutine("DT Neutral Score Climb", this::dtnClimb);

    RobotModeTriggers.autonomous().whileTrue(autoChooserLOL.selectedCommandScheduler());
  }

  public void configDashboard(ShuffleboardTab tab) {
    tab.add("auto chooser lol", autoChooserLOL).withWidget(BuiltInWidgets.kComboBoxChooser);
    //tab.addDouble("Dis Sensor Values", () -> disSensor.getDistance().refresh().getValueAsDouble()).withWidget(BuiltInWidgets.kTextView);
  }

  private SequentialCommandGroup shootSequence() {
    return new SequentialCommandGroup(
        new AimHub(m_robotContainer, m_robotContainer.visabelle).withTimeout(0.5),
        new ParallelCommandGroup( 
          new SequentialCommandGroup( 
              new ParallelCommandGroup(
                  new SequentialCommandGroup(
                      new RunYeeter(m_robotContainer.yeeter, () -> (m_robotContainer.yeeter.getNecessarySpeed(() -> m_robotContainer.visabelle.getDisFromHub()) + Constants.Yeeter.YEETER_SPEED_ADDITION), Constants.Yeeter.YEETER_ACCELERATION).withTimeout(3),  // Constants.Yeeter.YEETER_SPEED + 4.0, Constants.Yeeter.YEETER_ACCELERATION), // TODO: test timeout

                      new RunYeeter(m_robotContainer.yeeter, () -> (m_robotContainer.yeeter.getNecessarySpeed(() -> m_robotContainer.visabelle.getDisFromHub())), Constants.Yeeter.YEETER_ACCELERATION) // Constants.Yeeter.YEETER_SPEED, Constants.Yeeter.YEETER_ACCELERATION)
                  ),
                  new SequentialCommandGroup(
                      new WaitUntilCommand(() -> m_robotContainer.yeeter.reachedYeeterSpeed(true)), //now this will check for the higher speed TODO: test if the balls start feeding within the 3 sec and if there is any cases they don't

                      new RunFeeder(m_robotContainer.feeder, Constants.Feeder.PREP_FEEDER).withTimeout(0.5),
                      
                      new ParallelCommandGroup(
                          new RunFeeder(m_robotContainer.feeder, Constants.Feeder.FEEDER_SPEED),
                          new RunIndexer(m_robotContainer.indexer, Constants.Indexer.INDEXER_SPEED),

                          new SequentialCommandGroup(
                              new WaitCommand(1.0), 
                              new MovePivot(m_robotContainer.pivot, Constants.Pivot.SLIGHTLY_UP_FROM_DOWN, true),
                              new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION, false).withTimeout(1),
                              new MovePivot(m_robotContainer.pivot, Constants.Pivot.SLIGHTLY_UP_FROM_DOWN, false),
                              new RunEater(m_robotContainer.eater, Constants.Eater.EATER_MOTOR_SPEED).withTimeout(2)
                          )
                      )
                  )
              )
          )
          // new RunCommand(() -> swerve.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0)
          //     .withRotationalRate(Math.sin(Timer.getFPGATimestamp() * 10) * MaxAngularRate * 0.3)), swerve)
        ).withTimeout(6)
      );
  }

  private SequentialCommandGroup intakeSequence() {
    return new SequentialCommandGroup(new ParallelCommandGroup(
      new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION, false), //wasnt there before
      new RunEater(m_robotContainer.eater, Constants.Eater.EATER_MOTOR_SPEED)
    ));
  }

  private AutoRoutine hubScore() {
    AutoRoutine routine = autoFactory.newRoutine("HUBSCORE");
    AutoTrajectory goBack = routine.trajectory("ShootFromABitBack");
    AutoTrajectory score  = routine.trajectory("H_Shoot");
    double initialOrientation = goBack.getInitialPose().get().getRotation().getDegrees();

    // WITHOUT EVENT MARKER
    routine.active().onTrue(
      Commands.sequence(
          new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
          //step one: set gyro to starting heading (flips for alliance)
          new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
          new PrintCommand("!!!!!***** gyro set to starting heading"),

          goBack.resetOdometry(),

          //step three: set LL heading to gyro (aka starting) heading
          new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
          new PrintCommand("!!!!!***** LL heading set to gyro heading"),
          
          goBack.cmd()
      )
    );

    goBack.done().onTrue(
      score.cmd().alongWith(new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION, false))
    );
    // score.active().onTrue(
    //   new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION)
    // );
    score.done().onTrue(
       shootSequence());
    return routine;
  }
  
  private AutoRoutine oto() {
    AutoRoutine routine = autoFactory.newRoutine("OT_O");
    AutoTrajectory otToO = routine.trajectory("OT_O");
    AutoTrajectory turnAndShoot = routine.trajectory("O_Shoot");
    double initialOrientation = otToO.getInitialPose().get().getRotation().getDegrees();

    routine.active().onTrue(
      Commands.sequence(
        new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
        //step one: set gyro to starting heading (flips for alliance)
         new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
        new PrintCommand("!!!!!***** gyro set to starting heading"),

        otToO.resetOdometry(),
        new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
  
        otToO.cmd()
      )
    ); 

    otToO.active().onTrue(intakeSequence());

    otToO.done().onTrue(turnAndShoot.cmd());
   
    turnAndShoot.done().onTrue(
      shootSequence());
    return routine;
  }

private AutoRoutine hTo() { // hub to outpost go a little forward shoot
    AutoRoutine routine = autoFactory.newRoutine("hTO");
    AutoTrajectory hubToO = routine.trajectory("H_O");
    AutoTrajectory Shoot = routine.trajectory("O_Shoot");
    double initialOrientation = hubToO.getInitialPose().get().getRotation().getDegrees();

    routine.active().onTrue(
      Commands.sequence(
        new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
        //step one: set gyro to starting heading (flips for alliance)
         new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
        new PrintCommand("!!!!!***** gyro set to starting heading"),

        hubToO.resetOdometry(),
        new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
  
        hubToO.cmd()
      )
    ); 

    hubToO.active().onTrue(
      intakeSequence()
    );

    hubToO.done().onTrue(Shoot.cmd());
   
    Shoot.done().onTrue(shootSequence());

    return routine;
  }
private AutoRoutine hTd() { // hub to depot go a little forward shoot
    AutoRoutine routine = autoFactory.newRoutine("hTO");
    AutoTrajectory hubToD = routine.trajectory("H_D");
    AutoTrajectory Shoot = routine.trajectory("D_Shoot");
    double initialOrientation = hubToD.getInitialPose().get().getRotation().getDegrees();

    routine.active().onTrue(
      Commands.sequence(
        new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
        //step one: set gyro to starting heading (flips for alliance)
         new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
        new PrintCommand("!!!!!***** gyro set to starting heading"),

        hubToD.resetOdometry(),
        new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
  
        hubToD.cmd()
      )
    ); 

    hubToD.active().onTrue(
      intakeSequence()
    );

    hubToD.done().onTrue(Shoot.cmd());
   
    Shoot.done().onTrue(
      shootSequence());

    return routine;
  }

  private AutoRoutine hotn() { // hub to outpost to trench to neutral to shoot
    AutoRoutine routine = autoFactory.newRoutine("hubToOutpostToTrenchToNeutral");
    AutoTrajectory hubToO = routine.trajectory("H_O");
    AutoTrajectory Shoot = routine.trajectory("O_Shoot");//change choreo name 
    AutoTrajectory TrenchNeutral= routine.trajectory("TrenchNeutralOT"); //change choreo name
    AutoTrajectory NeutralIntake= routine.trajectory("N_Intake"); //change choreo name
    AutoTrajectory goBackShoot = routine.trajectory("intakeShoot"); //change choreo name 
    double initialOrientation = hubToO.getInitialPose().get().getRotation().getDegrees();

    routine.active().onTrue(
      Commands.sequence(
        new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
        //step one: set gyro to starting heading (flips for alliance)
         new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
        new PrintCommand("!!!!!***** gyro set to starting heading"),

        hubToO.resetOdometry(),
        new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
  
        hubToO.cmd()
      )
    ); 

    hubToO.active().onTrue(
      intakeSequence()
    );

    hubToO.done().onTrue(Shoot.cmd());
   
    Shoot.done().onTrue(
      shootSequence()
      .andThen(TrenchNeutral.cmd()));
    

    TrenchNeutral.done().onTrue(
      new RunEater(m_robotContainer.eater, Constants.Eater.EATER_MOTOR_SPEED)
    );
    NeutralIntake.active().onTrue(
      new RunEater(m_robotContainer.eater, Constants.Eater.EATER_MOTOR_SPEED)
    );
    TrenchNeutral.done().onTrue(NeutralIntake.cmd());
    NeutralIntake.done().onTrue(goBackShoot.cmd());

    goBackShoot.done().onTrue(
      shootSequence()
      ); 

    return routine;
  }

   private AutoRoutine hdtn() { // hub to depot to trench to neutral to shoot
    AutoRoutine routine = autoFactory.newRoutine("hubToOutpostToTrenchToNeutral");
    AutoTrajectory hubToD = routine.trajectory("H_D");
    AutoTrajectory Shoot = routine.trajectory("D_Shoot");//change choreo name 
    AutoTrajectory TrenchNeutral= routine.trajectory("TrenchNeutralDT"); //change choreo name
    AutoTrajectory NeutralIntake= routine.trajectory("DT_N_Intake");
    AutoTrajectory goBackShoot = routine.trajectory("goBackDT"); //change choreo name 
    double initialOrientation = hubToD.getInitialPose().get().getRotation().getDegrees();

    routine.active().onTrue(
      Commands.sequence(
        new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
        //step one: set gyro to starting heading (flips for alliance)
         new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
        new PrintCommand("!!!!!***** gyro set to starting heading"),

        hubToD.resetOdometry(),
        new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
  
        hubToD.cmd()
      )
    ); 

    hubToD.active().onTrue(
      intakeSequence()
    );

    hubToD.done().onTrue(Shoot.cmd());
   
    Shoot.done().onTrue(
      shootSequence()
      .andThen(TrenchNeutral.cmd()));

     TrenchNeutral.done().onTrue(
      new RunEater(m_robotContainer.eater, Constants.Eater.EATER_MOTOR_SPEED)
    );
    NeutralIntake.active().onTrue(
      new RunEater(m_robotContainer.eater, Constants.Eater.EATER_MOTOR_SPEED)
    );
    TrenchNeutral.done().onTrue(NeutralIntake.cmd());
    NeutralIntake.done().onTrue(goBackShoot.cmd());


    goBackShoot.done().onTrue(
      shootSequence()
    );

    return routine;
  }

  
  private AutoRoutine dtn2x() {
    AutoRoutine routine = autoFactory.newRoutine("DTN2X");

    AutoTrajectory trenchToCenter = routine.trajectory("DT_N");
    AutoTrajectory intakeMore1  = routine.trajectory("DT_N_fuelBranch1");
    AutoTrajectory goBack1 = routine.trajectory("N_DTShoot1");
    AutoTrajectory shootToCenter = routine.trajectory("Shoot_DT_N2");
    AutoTrajectory intakeMore2 = routine.trajectory("DT_N_fuelBranch2");
    AutoTrajectory goBack2 = routine.trajectory("N_DTShoot2");

    double initialOrientation = trenchToCenter.getInitialPose().get().getRotation().getDegrees();

    routine.active().onTrue(
      Commands.sequence(
          new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
          //step one: set gyro to starting heading (flips for alliance)
          new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
          new PrintCommand("!!!!!***** gyro set to starting heading"),

          trenchToCenter.resetOdometry(),

          //step three: set LL heading to gyro (aka starting) heading
          new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
          new PrintCommand("!!!!!***** LL heading set to gyro heading"),
          
          trenchToCenter.cmd(),
          new PrintCommand("auto start")
      )
    );

    trenchToCenter.active().onTrue(
      intakeSequence()
    );
    trenchToCenter.done().onTrue(intakeMore1.cmd());
    intakeMore1.active().onTrue(
      intakeSequence()
    );
    intakeMore1.done().onTrue(goBack1.cmd());

    goBack1.done().onTrue(
      shootSequence()
      .andThen(shootToCenter.cmd()));

    shootToCenter.done().onTrue(intakeMore2.cmd());
    intakeMore2.active().onTrue(
      new RunEater(m_robotContainer.eater, Constants.Eater.EATER_MOTOR_SPEED)
    );
    intakeMore2.done().onTrue(goBack2.cmd());
    goBack2.done().onTrue(shootSequence());

    //finish the first path and get to the intaking pose. if our distance sensor detects fuel
    //the hopper is full, so we should continue with the rest of the auto and go shoot
    // Trigger atNeutral = trenchToCenter.done();
    // atNeutral.and(()-> disSensor.getDistance().getValueAsDouble() >= 27).onTrue(intakeMore.cmd());//if true then intake 
    // //  //write intake for fuel traj if true 
    // atNeutral.and(()-> disSensor.getDistance().getValueAsDouble() < 27).onTrue(shootClimb.cmd());

    return routine;
  }
  private AutoRoutine dtndisrupt() {
    AutoRoutine routine = autoFactory.newRoutine("DTNDISRUPT");

    AutoTrajectory path1 = routine.trajectory("DT_Ndisrupt");
    AutoTrajectory path2  = routine.trajectory("DT_Ndisrupt1");
    AutoTrajectory path3 = routine.trajectory("DT_Ndisrupt2");
    AutoTrajectory path4 = routine.trajectory("DT_Ndisrupt3");
    AutoTrajectory path5 = routine.trajectory("DT_Ndisrupt4");
    AutoTrajectory path6 = routine.trajectory("DT_Ndisrupt5");
    

    double initialOrientation = path1.getInitialPose().get().getRotation().getDegrees();

    routine.active().onTrue(
      Commands.sequence(
          new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
          //step one: set gyro to starting heading (flips for alliance)
          new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
          new PrintCommand("!!!!!***** gyro set to starting heading"),

          path1.resetOdometry(),

          //step three: set LL heading to gyro (aka starting) heading
          new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
          new PrintCommand("!!!!!***** LL heading set to gyro heading"),
          
          path1.cmd()
      )
    );

    path1.active().onTrue(
     intakeSequence());
    
    path1.done().onTrue(path2.cmd());
    path2.active().onTrue(
      intakeSequence());
    path2.done().onTrue(path3.cmd()); //TODO: test if as we go back from neutral zone, are there fuel we can intake?

    path3.done().onTrue( //TODO: test if starting shooting from the trench pos results in missed balls
      shootSequence().andThen(path4.cmd())
    );

    path4.done().onTrue(path5.cmd());
    path5.active().onTrue(
      intakeSequence());
    path5.done().onTrue(path6.cmd());
    path6.done().onTrue(shootSequence());

    return routine;
  }
  private AutoRoutine otdisrupt() {
    AutoRoutine routine = autoFactory.newRoutine("OTDISRUPT");

    AutoTrajectory path1 = routine.trajectory("OT_disrupt");
    AutoTrajectory path2  = routine.trajectory("OT_disrupt1");
    AutoTrajectory path3 = routine.trajectory("OT_disrupt2");
    AutoTrajectory path4 = routine.trajectory("OT_disrupt3");
    AutoTrajectory path5 = routine.trajectory("OT_disrupt4");
    AutoTrajectory path6 = routine.trajectory("OT_disrupt5");
    

    double initialOrientation = path1.getInitialPose().get().getRotation().getDegrees();

    routine.active().onTrue(
      Commands.sequence(
          new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
          //step one: set gyro to starting heading (flips for alliance)
          new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
          new PrintCommand("!!!!!***** gyro set to starting heading"),

          path1.resetOdometry(),

          //step three: set LL heading to gyro (aka starting) heading
          new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
          new PrintCommand("!!!!!***** LL heading set to gyro heading"),
          
          path1.cmd(),
          new PrintCommand("auto start")
      )
    );
    path1.active().onTrue(
      intakeSequence());

    path1.done().onTrue(path2.cmd());

    path2.active().onTrue(
      intakeSequence()
    );

    path2.done().onTrue(path3.cmd());
    
    path3.done().onTrue(shootSequence().andThen(path4.cmd()));

    path4.active().onTrue(intakeSequence());

    path4.done().onTrue(path5.cmd());

    path5.active().onTrue(intakeSequence());
    
    path5.done().onTrue(path6.cmd());

    path6.done().onTrue(shootSequence());

    return routine;
  }

  private AutoRoutine dtnDisSensor() {
    AutoRoutine routine = autoFactory.newRoutine("DT Neutral Zone Distance Sensor");

    AutoTrajectory trenchToCenter = routine.trajectory("DT_N_DisSensor");
    AutoTrajectory intake  = routine.trajectory("DT_N_intake_first_time");
    AutoTrajectory shootFirstCondition = routine.trajectory("N_DTShoot2_enough_balls");
    AutoTrajectory intakeMore = routine.trajectory("DT_N_fuelBranch2_disSensor");
    AutoTrajectory shootSecondCondition = routine.trajectory("N_DTShoot2_not_enough");

    double initialOrientation = trenchToCenter.getInitialPose().get().getRotation().getDegrees();

    routine.active().onTrue(
      Commands.sequence(
          new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
          //step one: set gyro to starting heading (flips for alliance)
          new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
          new PrintCommand("!!!!!***** gyro set to starting heading"),

          trenchToCenter.resetOdometry(),

          //step three: set LL heading to gyro (aka starting) heading
          new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
          new PrintCommand("!!!!!***** LL heading set to gyro heading"),
          
          trenchToCenter.cmd(),
          new PrintCommand("auto start")
      )
    );

    trenchToCenter.active().onTrue(
      intakeSequence()
    );
    trenchToCenter.done().onTrue(intake.cmd());
    
    intake.active().onTrue(
      new RunEater(m_robotContainer.eater, Constants.Eater.EATER_MOTOR_SPEED));

    intake.done().onTrue(shootFirstCondition.cmd());

    shootFirstCondition.done().onTrue(shootSequence());

    intakeMore.active().onTrue(
      new RunEater(m_robotContainer.eater, Constants.Eater.EATER_MOTOR_SPEED)
    );

    intakeMore.done().onTrue(shootSecondCondition.cmd());

    shootSecondCondition.done().onTrue(shootSequence());

    return routine;
  }
  private AutoRoutine dtn2xBump() {
    AutoRoutine routine = autoFactory.newRoutine("DT Neutral Zone Distance Sensor");

    AutoTrajectory trenchToCenter = routine.trajectory("DT_N");
    AutoTrajectory intake1  = routine.trajectory("DT_N_fuelBranch1");
    AutoTrajectory shoot1 = routine.trajectory("bumpN_DTShoot1");
    AutoTrajectory goBack = routine.trajectory("bumpShoot_DT_N");
    AutoTrajectory intake2 = routine.trajectory("bumpDT_N_fuelBranch2");
    AutoTrajectory shoot2 = routine.trajectory("bumpN_DTShoot2");
    double initialOrientation = trenchToCenter.getInitialPose().get().getRotation().getDegrees();

    routine.active().onTrue(
      Commands.sequence(
          new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
          //step one: set gyro to starting heading (flips for alliance)
          new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
          new PrintCommand("!!!!!***** gyro set to starting heading"),

          trenchToCenter.resetOdometry(),

          //step three: set LL heading to gyro (aka starting) heading
          new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
          new PrintCommand("!!!!!***** LL heading set to gyro heading"),
          
          trenchToCenter.cmd(),
          new PrintCommand("auto start")
      )
    );

    trenchToCenter.active().onTrue(
      intakeSequence());
      
    trenchToCenter.done().onTrue(intake1.cmd());
    
    intake1.active().onTrue(
     intakeSequence());    
     
    intake1.done().onTrue(shoot1.cmd());

    shoot1.done().onTrue(
      shootSequence().andThen(goBack.cmd()));
    
    goBack.done().onTrue(intake2.cmd());
    
    intake2.active().whileTrue(
      intakeSequence());
    
    intake2.done().onTrue(shoot2.cmd());
  
    shoot2.done().onTrue(shootSequence());
      
    return routine;
  }
  private AutoRoutine dtn2xBumpDelay() {
    AutoRoutine routine = autoFactory.newRoutine("DT Neutral Zone Distance Sensor");

    AutoTrajectory trenchToCenter = routine.trajectory("DT_N");
    AutoTrajectory intake1  = routine.trajectory("DT_N_fuelBranch1");
    AutoTrajectory shoot1 = routine.trajectory("bumpN_DTShoot1");
    AutoTrajectory goBack = routine.trajectory("bumpShoot_DT_N");
    AutoTrajectory intake2 = routine.trajectory("bumpDT_N_fuelBranch2");
    AutoTrajectory shoot2 = routine.trajectory("bumpN_DTShoot2");
    double initialOrientation = trenchToCenter.getInitialPose().get().getRotation().getDegrees();

    routine.active().onTrue(
      Commands.sequence(
          new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
          //step one: set gyro to starting heading (flips for alliance)
          new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
          new PrintCommand("!!!!!***** gyro set to starting heading"),

          trenchToCenter.resetOdometry(),

          //step three: set LL heading to gyro (aka starting) heading
          new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
          new PrintCommand("!!!!!***** LL heading set to gyro heading"),
          new WaitCommand(2.5),
          trenchToCenter.cmd(),
          new PrintCommand("auto start")
      )
    );

    trenchToCenter.active().onTrue(
      intakeSequence()
    );
    trenchToCenter.done().onTrue(intake1.cmd());
    
    intake1.active().onTrue(
     intakeSequence());    
      intake1.done().onTrue(shoot1.cmd());

  
    shoot1.done().onTrue(shootSequence().andThen(goBack.cmd()));
      goBack.active().onTrue(
      intakeSequence()
    );
    goBack.done().onTrue(intake2.cmd());
    
    intake2.active().onTrue(
      intakeSequence());
          intake2.done().onTrue(shoot2.cmd());

  
    shoot2.done().onTrue(shootSequence());
      
    return routine;
  }


    private AutoRoutine otn2x() {
    AutoRoutine routine = autoFactory.newRoutine("OTN2X");

    AutoTrajectory trenchNeutral = routine.trajectory("OT_N");
    AutoTrajectory intake1  = routine.trajectory("OT_N_fuelBranch1");
    AutoTrajectory shoot1 = routine.trajectory("N_OTShoot1");
    AutoTrajectory goBack = routine.trajectory("OT_Shoot_N");
    AutoTrajectory intake2 = routine.trajectory("OT_N_fuelBranch2");
    AutoTrajectory shoot2 = routine.trajectory("N_OTShoot2");

    double initialOrientation = trenchNeutral.getInitialPose().get().getRotation().getDegrees();
    
    routine.active().onTrue(
      Commands.sequence(
          new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
          //step one: set gyro to starting heading (flips for alliance)
          new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
          new PrintCommand("!!!!!***** gyro set to starting heading"),

          trenchNeutral.resetOdometry(),

          //step three: set LL heading to gyro (aka starting) heading
          new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
          new PrintCommand("!!!!!***** LL heading set to gyro heading"),
          
          trenchNeutral.cmd()
      )
    );

    trenchNeutral.done().onTrue(
      new ParallelCommandGroup(
            new MovePivot(m_robotContainer.pivot, Constants.Pivot.DOWN_POSITION, false)
      )
    );
    trenchNeutral.done().onTrue(intake1.cmd());
    intake1.active().onTrue(
      new RunEater(m_robotContainer.eater, Constants.Eater.EATER_MOTOR_SPEED)
    );
    intake1.done().onTrue(shoot1.cmd()); //TODO: test if as we go back from neutral zone, are there fuel we can intake?

    shoot1.done().onTrue( //TODO: test if starting shooting from the trench pos results in missed balls
      shootSequence()
      .andThen(goBack.cmd())
    );

    goBack.done().onTrue(intake2.cmd());
    intake2.active().whileTrue(
      new RunEater(m_robotContainer.eater, Constants.Eater.EATER_MOTOR_SPEED)
    );
    intake2.done().onTrue(shoot2.cmd());
    shoot2.done().onTrue(shootSequence());
    //finish the first path and get to the intaking pose. if our distance sensor detects fuel
    //the hopper is full, so we should continue with the rest of the auto and go shoot
    // Trigger atNeutral = trenchToCenter.done();
    // atNeutral.and(()-> disSensor.getDistance().getValueAsDouble() >= 27).onTrue(intakeMore.cmd());//if true then intake 
    // //  //write intake for fuel traj if true 
    // atNeutral.and(()-> disSensor.getDistance().getValueAsDouble() < 27).onTrue(shootClimb.cmd());

    return routine;
  }
  private AutoRoutine otn2xBump() {
    AutoRoutine routine = autoFactory.newRoutine("OTN2X");

    AutoTrajectory trenchNeutral = routine.trajectory("OT_N");
    AutoTrajectory intake1  = routine.trajectory("OT_N_fuelBranch1");
    AutoTrajectory shoot1 = routine.trajectory("bumpN_OTShoot1");
    AutoTrajectory goBack = routine.trajectory("bumpShoot_OT_N");
    AutoTrajectory intake2 = routine.trajectory("bumpOT_N_fuelBranch2");
    AutoTrajectory shoot2 = routine.trajectory("bumpN_OTShoot2");

    double initialOrientation = trenchNeutral.getInitialPose().get().getRotation().getDegrees();
    
    routine.active().onTrue(
      Commands.sequence(
          new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
          //step one: set gyro to starting heading (flips for alliance)
          new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
          new PrintCommand("!!!!!***** gyro set to starting heading"),

          trenchNeutral.resetOdometry(),

          //step three: set LL heading to gyro (aka starting) heading
          new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
          new PrintCommand("!!!!!***** LL heading set to gyro heading"),
          
          trenchNeutral.cmd()
      )
    );

    trenchNeutral.active().onTrue(
     intakeSequence());
    
    trenchNeutral.done().onTrue(intake1.cmd());
    intake1.active().onTrue(
      intakeSequence());
    intake1.done().onTrue(shoot1.cmd()); //TODO: test if as we go back from neutral zone, are there fuel we can intake?

    shoot1.done().onTrue( //TODO: test if starting shooting from the trench pos results in missed balls
      shootSequence().andThen(goBack.cmd())
    );

    goBack.done().onTrue(intake2.cmd());
    intake2.active().onTrue(
      intakeSequence());
    intake2.done().onTrue(shoot2.cmd());
    shoot2.done().onTrue(shootSequence());

    return routine;
  }

  private AutoRoutine htw() {
    AutoRoutine routine = autoFactory.newRoutine("HTW");
    AutoTrajectory shootFromABitBack = routine.trajectory("H_Shoot");
    AutoTrajectory hubTowerShoot = routine.trajectory("H_TW");
    double initialOrientation = shootFromABitBack.getInitialPose().get().getRotation().getDegrees();

    routine.active().onTrue(
      Commands.sequence(
        new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
        //step one: set gyro to starting heading (flips for alliance)
        new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
        new PrintCommand("!!!!!***** gyro set to starting heading"),

        shootFromABitBack.resetOdometry(),
        new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
        //step three: set LL heading to gyro (aka starting) heading
        //if we can see a tag then run step 3 & path else just run path
        shootFromABitBack.cmd()
      )
    ); 
    shootFromABitBack.active().onTrue(
      intakeSequence()
    );

    shootFromABitBack.done().onTrue(
      shootSequence()
        .andThen(hubTowerShoot.cmd())
      );
      hubTowerShoot.active().onTrue(
        new MoveClimbUp(m_robotContainer.climb, -15)
      );
      hubTowerShoot.done().onTrue(
        new SequentialCommandGroup(
          new MoveClimbUp(m_robotContainer.climb, -15),
          new AlignTowerPose(m_robotContainer.swerve).withTimeout(3),
          new MoveClimbtoZero(m_robotContainer.climb, 15)
        )
      );
    return routine;
  }
private AutoRoutine dtnClimb() {
    AutoRoutine routine = autoFactory.newRoutine("DTNCLIMB");
    AutoTrajectory trenchToCenter = routine.trajectory("DT_N");
    AutoTrajectory intake = routine.trajectory("DT_N_fuelBranch1");
    AutoTrajectory goBack = routine.trajectory("N_DTShoot1");
    AutoTrajectory climb = routine.trajectory("DT_Shoot_Climb");
    double initialOrientation = trenchToCenter.getInitialPose().get().getRotation().getDegrees();

    routine.active().onTrue(
      Commands.sequence(
        new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
        //step one: set gyro to starting heading (flips for alliance)
        new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
        new PrintCommand("!!!!!***** gyro set to starting heading"),

        trenchToCenter.resetOdometry(),
        new InstantCommand(
            () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
            m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
            0, 0, 0, 0, 0)
          ),
        //step three: set LL heading to gyro (aka starting) heading
        //if we can see a tag then run step 3 & path else just run path
        trenchToCenter.cmd()
      )
    ); 
    trenchToCenter.active().onTrue(
      intakeSequence()
    );

    trenchToCenter.done().onTrue(intake.cmd()
      );
    intake.active().onTrue(intakeSequence());
    intake.done().onTrue(goBack.cmd());
    goBack.done().onTrue(shootSequence().andThen(climb.cmd()));
      climb.active().onTrue(
        new MoveClimbUp(m_robotContainer.climb, -15)
      );
      climb.done().onTrue(
        new SequentialCommandGroup(
          new MoveClimbUp(m_robotContainer.climb, -15),
          new AlignTowerPose(m_robotContainer.swerve).withTimeout(3),
          new MoveClimbtoZero(m_robotContainer.climb, 15)
        )
      );
    return routine;
  }
private AutoRoutine towerTest() {
  AutoRoutine routine = autoFactory.newRoutine("TWtest");
  AutoTrajectory path1 = routine.trajectory("towertest");
  double initialOrientation = path1.getInitialPose().get().getRotation().getDegrees();

  routine.active().onTrue(
    Commands.sequence(
      new PrintCommand("!!!!!***** initial orientation has been gotten from start pose"),
      //step one: set gyro to starting heading (flips for alliance)
      new InstantCommand(() -> m_robotContainer.swerve.getPigeon2().setYaw(initialOrientation)),
      new PrintCommand("!!!!!***** gyro set to starting heading"),

      path1.resetOdometry(),
      new InstantCommand(
          () -> LimelightHelpers.SetRobotOrientation("limelight-front", 
          m_robotContainer.swerve.getPigeon2().getRotation2d().getDegrees(), 
          0, 0, 0, 0, 0)
        ),
      //step three: set LL heading to gyro (aka starting) heading
      //if we can see a tag then run step 3 & path else just run path
      path1.cmd()
    )
  ); 
  path1.active().onTrue(
    new MoveClimbUp(m_robotContainer.climb, -15)
  );
    path1.done().onTrue(
      new SequentialCommandGroup(
        new MoveClimbUp(m_robotContainer.climb, -15),
        new AlignTowerPose(m_robotContainer.swerve).withTimeout(3),
        new MoveClimbtoZero(m_robotContainer.climb, 15)
      )
    );
  return routine;
}
}