// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import java.util.Optional;

import com.ctre.phoenix6.CANBus; 
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    //drivetrain
        public final Swerve swerve = TunerConstants.createDrivetrain();
        public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        private final Telemetry logger = new Telemetry(MaxSpeed);
    
        /* Setting up bindings for necessary control of the swerve drive platform */
        public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        
        public final SwerveRequest.FieldCentricFacingAngle driveAtAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1) // 0.1 = deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //mechanism
        public static final CANBus CANBus = new CANBus("Default Name");
        public final IntakeShooter intake = new IntakeShooter(this);
        public final Feeder feeder = new Feeder();
        public final IntakeShooter shooter = new IntakeShooter(this);
        public Autoes autoes = new Autoes(this);
        public double shooterSpeed = 0.0;

        //public DoubleSupplierSubscriber speedTunable = DogLog.tunable("Tunable Speed", () -> () -> Constants.Shooter.SHOOTER_SPEED);
        //public DoubleSubscriber angleTunable = DogLog.tunable("Tunable Angle", Constants.Hood.HOOD_ANGLE);

    //control
        public final CommandXboxController m_driverController = new CommandXboxController(0);
        public final CommandXboxController m_operatorController = new CommandXboxController(1);

        private Optional<Alliance> ally; 
    
        public ShuffleboardTab fieldTab = Shuffleboard.getTab("Field"); 
        public final ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
        //public static ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");
        
        private boolean hasAlreadyUpdatedIfWeWonAuto = false;
        public final Trigger updateWinAuto = new Trigger(() -> hasAlreadyUpdatedIfWeWonAuto);

    public RobotContainer() {
        configureBindings();
        autoes.configDashboard(matchTab);
        //shooter.configDashboard(matchTab);
  
        // Schedule the selected auto during the autonomous period
        // matchTab.add("auto chooser LOL", autoChooserLOL).withWidget(BuiltInWidgets.kComboBoxChooser);
        ally = DriverStation.getAlliance(); 
    
        //for vision servoing
        driveAtAngle.HeadingController.setPID(5, 0.0, 0.0); //TODO: took PID from tuner constants, need to check
        driveAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }
    

    private void configureBindings() {
        //SWERVE
            // Note that X is defined as forward according to WPILib convention,
            // and Y is defined as to the left according to WPILib convention.
            swerve.setDefaultCommand(
                // Drivetrain will execute this command periodically
                swerve.applyRequest(() ->
                    drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
            );

            /* reset gyro */
            m_driverController.start().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));//.seedFieldCentric()

            //POV buttons
            m_driverController.povUp().whileTrue(swerve.applyRequest(() ->
            drive.withVelocityX(0.2 * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

            m_driverController.povDown().whileTrue(swerve.applyRequest(() ->
            drive.withVelocityX(-0.2 * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

            m_driverController.povRight().whileTrue(swerve.applyRequest(() ->
            drive.withVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-0.2 * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

            m_driverController.povLeft().whileTrue(swerve.applyRequest(() ->
            drive.withVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(0.2 * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

            // Idle while the robot is disabled. This ensures the configured
            // neutral mode is applied to the drive motors while disabled.
            final var idle = new SwerveRequest.Idle();
            RobotModeTriggers.disabled().whileTrue(
                swerve.applyRequest(() -> idle).ignoringDisable(true)
            );

            swerve.registerTelemetry(logger::telemeterize);

            /* defense mode */
            m_driverController.leftBumper().whileTrue(swerve.applyRequest(() -> brake));
            
            // m_driverController.x().whileTrue(
            //     new ConditionalCommand(new RunCommand(() -> gerryRig.runMotor(0.7), gerryRig),
            //         new RunCommand(() -> gerryRig.stopMotor(), gerryRig), 
            //         () -> autoes.getDisSensor() <= 0.15)
            // );

            // Run SysId routines when holding back/start and X/Y.
            // Note that each routine should be run exactly once in a single log.
            m_driverController.povDown().and(m_driverController.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
            m_driverController.povDown().and(m_driverController.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
            m_driverController.povUp().and(m_driverController.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
            m_driverController.povUp().and(m_driverController.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

            //check for winning auto
            updateWinAuto.whileFalse(new RunCommand(() -> wonAuto(matchTab)));

        //VISION
            // hub alignment but with localization
            m_driverController.rightTrigger().whileTrue(
                swerve.applyRequest(() ->
                    driveAtAngle.withTargetDirection(new Rotation2d())
                        .withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                )
            );

            //snap to hub
            m_driverController.rightBumper().whileTrue(
                swerve.applyRequest(() ->
                    driveAtAngle.withTargetDirection(Rotation2d.kPi)
                        .withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                )
            );

            //align to tower
            //m_driverController.leftTrigger().whileTrue(new AlignTowerPose(swerve));

            // yaw setter --> 0 faces hub 
            m_driverController.x().onTrue(new SequentialCommandGroup(
                // ROTATION2D IS IN **RADIANS!!!!**
                // SET YAW IS IN **DEGREES!!!!**
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> swerve.setOperatorPerspectiveForward(Rotation2d.kPi)),
                        new InstantCommand(() -> swerve.getPigeon2().setYaw(180.0)),
                        new InstantCommand(() -> swerve.getPigeon2().getYaw().waitForUpdate(0.1)),
                        new InstantCommand(() -> swerve.resetPose(new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(), Rotation2d.kPi)))        
                    ),
                    new SequentialCommandGroup(
                        new InstantCommand(() -> swerve.setOperatorPerspectiveForward(Rotation2d.kZero)),
                        new InstantCommand(() -> swerve.getPigeon2().setYaw(0.0)),
                        new InstantCommand(() -> swerve.getPigeon2().getYaw().waitForUpdate(0.1)),
                        new InstantCommand(() -> swerve.resetPose(new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(), Rotation2d.kZero)))
                    ),
                    () -> ally.get() == Alliance.Red
                )
            ));
        
        //MECHANISM DEFAULT COMMANDS
            //shooter.setDefaultCommand(new RunCommand(() -> shooter.stopMotor(), shooter));
            shooter.setDefaultCommand(new RunShooter(shooter, ()-> Constants.IntakeShooter.RESTING_SPEED, Constants.IntakeShooter.SHOOTER_ACCELERATION));

        //SHOOTER
            m_operatorController.leftTrigger().and(m_operatorController.povRight().negate()).whileTrue(
                new ParallelCommandGroup( 
                    new SequentialCommandGroup( 
                        new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                //new RunShooter(shooter, () -> (shooter.getNecessarySpeed(() -> vision.getDisFromHub()) + Constants.Shooter.SHOOTER_SPEED_ADDITION), Constants.Shooter.SHOOTER_ACCELERATION).withTimeout(3),  // Constants.Shooter.SHOOTER_SPEED + 4.0, Constants.Shooter.SHOOTER_ACCELERATION), // TODO: test timeout

                                //new RunShooter(shooter, () -> (shooter.getNecessarySpeed(() -> vision.getDisFromHub())), Constants.Shooter.SHOOTER_ACCELERATION) // Constants.Shooter.SHOOTER_SPEED, Constants.Shooter.SHOOTER_ACCELERATION)
                            ),
                            new SequentialCommandGroup(
                                new WaitUntilCommand(() -> shooter.reachedShooterSpeed(shooterSpeed)), //now this will check for the higher speed TODO: test if the balls start feeding within the 3 sec and if there is any cases they don't

                                new RunFeeder(feeder, Constants.Feeder.PREP_FEEDER).withTimeout(0.5),
                                
                                new ParallelCommandGroup(
                                    new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED),
                                    //new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED),

                                    new SequentialCommandGroup(
                                        new WaitCommand(1.0), 
                                        //new MovePivot(pivot, Constants.Pivot.SLIGHTLY_UP_FROM_DOWN, true),
                                        //new MovePivot(pivot, Constants.Pivot.DOWN_POSITION, false).withTimeout(0.5),
                                        //new MovePivot(pivot, Constants.Pivot.SLIGHTLY_UP_FROM_DOWN, false).withTimeout(0.5),
                                        new ParallelCommandGroup(
                                            // new SequentialCommandGroup(
                                            //     new MovePivot(pivot, Constants.Pivot.DOWN_POSITION, false).withTimeout(0.5),
                                            //     new MovePivot(pivot, Constants.Pivot.SAFE, false)    
                                            // ),
                                            new RunIntake(intake, Constants.IntakeShooter.INTAKE_MOTOR_SPEED)
                                        )
                                    )
                                )
                            )
                        )
                    )
                )
            );
            
            // eject shooter
            // m_operatorController.leftTrigger().and(m_operatorController.x()).whileTrue(
            //     new ParallelCommandGroup(
            //         new RunShooter(shooter, ()-> -Constants.Shooter.SHOOTER_SPEED, Constants.Shooter.SHOOTER_ACCELERATION),
            //         new RunCommand (() -> candle.setControl(magentaBlink))
            //     )
            // );


            //modified if we're stuck
            m_operatorController.leftTrigger().and(m_operatorController.povRight()).whileTrue(
                new SequentialCommandGroup(     
                        new ParallelCommandGroup(
                            new ParallelCommandGroup(
                                //new RunShooter(shooter, () -> shooter.getNecessarySpeed(() -> vision.getDisFromHub()), Constants.Shooter.SHOOTER_ACCELERATION) // Constants.Shooter.SHOOTER_SPEED, Constants.Shooter.SHOOTER_ACCELERATION) //() -> shooter.getNecessarySpeed(() -> vision.getDisFromHub())
                                //new RunCommand (() -> candle.setControl(yellowBlink))
                            ),
                            //new RunCommand(() -> ledSubsystem.runPattern(LEDPattern.solid(Color.kRed)).withName("Revving Up")), //TODO: update color                

                            new SequentialCommandGroup(
                                new WaitUntilCommand(() -> shooter.reachedShooterSpeed(shooterSpeed)), 
                                
                                // new ParallelCommandGroup( //green
                                //     new SequentialCommandGroup(
                                //         new RunCommand (() -> candle.setControl(redSolid)).withTimeout(1.0),
                                //         new RunCommand (() -> candle.setControl(whiteSolid)).withTimeout(1.0)
                                //     )
                                // ),

                                //new RunCommand(() -> ledSubsystem.runPattern(LEDPattern.solid(Color.kBlue)).withName("Shooting")), //TODO: update color
                                //new RunCommand (() -> candle.runColorFlowPattern(0, 0, 255)), //blue

                                new RunFeeder(feeder, Constants.Feeder.PREP_FEEDER).withTimeout(0.5)
                                
                                // new ParallelCommandGroup(
                                //     new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED),
                                //     new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED),

                                //     new SequentialCommandGroup(
                                //         new WaitCommand(0.5), 
                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_ONE, false).withTimeout(0.25),
                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_TWO, false),
                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_ONE, false).withTimeout(0.25),
                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_TWO, false),
                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_ONE, false).withTimeout(0.25),

                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_THREE, false).withTimeout(0.25),
                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_FOUR, false),
                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_THREE, false).withTimeout(0.25),
                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_FOUR, false),
                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_THREE, false).withTimeout(0.25),

                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_FIVE, false).withTimeout(0.25),
                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_SIX, false),
                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_FIVE, false).withTimeout(0.25),
                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_SIX, false),
                                //         new MovePivot(pivot, Constants.Pivot.JITTER_POS_FIVE, false).withTimeout(0.25)
                                //     )
                                // )
                            
                        )
                        //new MovePivot(pivot, Constants.Pivot.DOWN_POSITION)
                )
                )
            ); //TODO: add defense mode while the robot is shooting

        //SHUTTLING (FAR)
            m_operatorController.leftBumper().and(m_operatorController.rightTrigger().negate()).and(m_operatorController.x().negate()).whileTrue(
                new ParallelCommandGroup(
                    //new RunninTheHood(theHood, Constants.Hood.HOOD_MAX).withTimeout(0.5), 
                    new SequentialCommandGroup( 
                        new ParallelCommandGroup(
                            new ParallelCommandGroup(
                                new RunShooter(shooter, () -> Constants.IntakeShooter.SHOOTER_FAR_SHUTTLE, Constants.IntakeShooter.SHOOTER_ACCELERATION) // Constants.Shooter.SHOOTER_SPEED, Constants.Shooter.SHOOTER_ACCELERATION) //() -> shooter.getNecessarySpeed(() -> vision.getDisFromHub())
                                //new RunCommand (() -> candle.setControl(yellowBlink))
                            ),

                            new SequentialCommandGroup(
                                new WaitUntilCommand(() -> shooter.reachedShooterSpeed(shooterSpeed)),
                                new RunFeeder(feeder, Constants.Feeder.PREP_FEEDER).withTimeout(0.5),
                                new RunIntake(intake, Constants.IntakeShooter.INTAKE_MOTOR_SPEED)
                                // new ParallelCommandGroup(
                                //     new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED),
                                //     new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED),
                                //     new SequentialCommandGroup(
                                //         new WaitCommand(1.0), 
                                //         new MovePivot(pivot, Constants.Pivot.SLIGHTLY_UP_FROM_DOWN, true),
                                //         new MovePivot(pivot, Constants.Pivot.DOWN_POSITION, false).withTimeout(0.5),
                                //         new MovePivot(pivot, Constants.Pivot.SLIGHTLY_UP_FROM_DOWN, false).withTimeout(0.5),
                                //         new ParallelCommandGroup(
                                //             // new SequentialCommandGroup(
                                //             //     new MovePivot(pivot, Constants.Pivot.DOWN_POSITION, false).withTimeout(0.5),
                                //             //     new MovePivot(pivot, Constants.Pivot.SAFE, false)
                                //             // ),
                                            
                                //         )
                                //     )
                                )
                            )
                        )
                    )
                ); 

        //SHUTTLING (SHORT)
            m_operatorController.leftBumper().and(m_operatorController.x()).and(m_operatorController.rightTrigger().negate()).whileTrue(
                new ParallelCommandGroup(
                    //new RunninTheHood(theHood, Constants.Hood.HOOD_MAX).withTimeout(0.5), 
                    new SequentialCommandGroup( 
                        new ParallelCommandGroup(
                            new ParallelCommandGroup(
                                new RunShooter(shooter, () -> Constants.IntakeShooter.SHOOTER_SPEED, Constants.IntakeShooter.SHOOTER_ACCELERATION) // Constants.Shooter.SHOOTER_SPEED, Constants.Shooter.SHOOTER_ACCELERATION) //() -> shooter.getNecessarySpeed(() -> vision.getDisFromHub())
                                //new RunCommand (() -> candle.setControl(yellowBlink))
                            ),

                            new SequentialCommandGroup(
                                new WaitUntilCommand(() -> shooter.reachedShooterSpeed(shooterSpeed)),
                                new RunFeeder(feeder, Constants.Feeder.PREP_FEEDER).withTimeout(0.5),
                                
                                new ParallelCommandGroup(
                                    new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED),
                                    //new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED),
                                    new SequentialCommandGroup(
                                        new WaitCommand(1.0), 
                                        // new MovePivot(pivot, Constants.Pivot.SLIGHTLY_UP_FROM_DOWN, true),
                                        // new MovePivot(pivot, Constants.Pivot.DOWN_POSITION, false).withTimeout(0.5),
                                        // new MovePivot(pivot, Constants.Pivot.SLIGHTLY_UP_FROM_DOWN, false).withTimeout(0.5),
                                        // new ParallelCommandGroup(
                                        //     new SequentialCommandGroup(
                                        //         new MovePivot(pivot, Constants.Pivot.DOWN_POSITION, false).withTimeout(0.5),
                                        //         new MovePivot(pivot, Constants.Pivot.SAFE, false)
                                        //     ),
                                            new RunIntake(intake, Constants.IntakeShooter.INTAKE_MOTOR_SPEED)
                                        )
                                    )
                                )
                            )
                        )
                    )
                ); 

        //SHUTTLE WHILE INTAKING
            m_operatorController.leftBumper().and(m_operatorController.rightTrigger()).and(m_operatorController.x().negate()).whileTrue(
                new ParallelCommandGroup(
                    // new ParallelCommandGroup(
                             //new RunninTheHood(theHood, Constants.Hood.HOOD_MAX).withTimeout(0.5), 
                        new SequentialCommandGroup( 
                            new ParallelCommandGroup(
                                new ParallelCommandGroup(
                                    new RunShooter(shooter, () -> Constants.IntakeShooter.SHOOTER_SPEED, Constants.IntakeShooter.SHOOTER_ACCELERATION) // Constants.Shooter.SHOOTER_SPEED, Constants.Shooter.SHOOTER_ACCELERATION) //() -> shooter.getNecessarySpeed(() -> vision.getDisFromHub())
                                    //new RunCommand (() -> candle.setControl(yellowBlink))
                                ),

                                new SequentialCommandGroup(
                                    new WaitUntilCommand(() -> shooter.reachedShooterSpeed(shooterSpeed)),
                                    new RunFeeder(feeder, Constants.Feeder.PREP_FEEDER).withTimeout(0.5),
                                    
                                    new ParallelCommandGroup(
                                        new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED)
                                        //new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED)

                                    )
                                )
                            )
                        )
                    )
                )  
            ;


        //FEEDER
            //isFeederStalling.whileTrue(new RunFeeder(feeder, 0.0));

            m_operatorController.rightBumper().whileTrue(
                new ParallelCommandGroup(
                    new RunFeeder(feeder, -100)
                )
            );


        //INTAKE
            m_operatorController.rightTrigger().and(m_operatorController.x().negate()).and(m_operatorController.leftBumper().negate()).whileTrue(
                new ParallelCommandGroup(
                    //new MovePivot(pivot, Constants.Pivot.DOWN_POSITION, false), //wasnt there before
                    new RunIntake(intake, Constants.IntakeShooter.INTAKE_MOTOR_SPEED)
                    //new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED),
                    //new RunFeeder(feeder, Constants.Feeder.INTAKE_FEEDER)
                    //new ConditionalCommand(new RunFeeder(feeder, 0), new RunFeeder(feeder, Constants.Feeder.INTAKE_FEEDER), ()-> feeder.isStalling()) //can change this back to just running it backwards if it doesnt work
                )
            );

            //eject
            m_operatorController.rightTrigger().and(m_operatorController.x()).and(m_operatorController.leftBumper().negate()).whileTrue(
                 new ParallelCommandGroup(  
                    new RunIntake(intake, -Constants.IntakeShooter.INTAKE_MOTOR_SPEED)
                )  
            );
            
            // m_operatorController.povRight().whileTrue(
            //      new ParallelCommandGroup(  
            //         new RunIntake(intake, 100),
            //         new RunCommand (() -> candle.setControl(whiteSolid))
            //     )  
            // );
    }

    public void wonAuto(ShuffleboardTab tab) {
        String gameData = DriverStation.getGameSpecificMessage();
        Alliance ourAlliance = DriverStation.getAlliance().get();
        Alliance winningAlliance = DriverStation.Alliance.Blue; //default

        if(gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B' :
                    winningAlliance = DriverStation.Alliance.Blue;
                    updateWonAuto();
                    break;
                case 'R' :
                    winningAlliance = DriverStation.Alliance.Red;
                    updateWonAuto();
                    break;
                default :
                    hasAlreadyUpdatedIfWeWonAuto = false;
                //This is corrupt data //TODO: what do we do here?
                break;
            }
        }

        boolean whoWon = ourAlliance.equals(winningAlliance);

        tab.addBoolean("won?", () -> whoWon)
            .withWidget(BuiltInWidgets.kBooleanBox).withPosition(7, 1)
            .withSize(2, 1);
    }

    private void updateWonAuto() {
        hasAlreadyUpdatedIfWeWonAuto = true;
    }
}
