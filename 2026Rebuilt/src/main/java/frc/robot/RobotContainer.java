// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.Joystick;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    //drivetrain
    public final SwerveOnTheseBows swerve = TunerConstants.createDrivetrain();
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final Telemetry logger = new Telemetry(MaxSpeed);
  
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //vision
    public Visabelle visabelle = new Visabelle(swerve, MaxAngularRate);
    public VisabelleUpdate visabelleUpdate = new VisabelleUpdate(swerve);
  
    //mechanism
    public static final CANBus CANBus = new CANBus("rio");
    public final Pivot pivot = new Pivot();
    public final Eater eater = new Eater();
    public final Indexer indexer = new Indexer();
    public final Feeder feeder = new Feeder();
    public final Yeeter yeeter = new Yeeter(this);
    public final TheHood theHood = new TheHood();
    public final Climb climb = new Climb();
    //public Autoes autoes = new Autoes(this);

    //control
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);
  
    public LED ledSubsystem = new LED();
    LEDPattern solidBlue = LEDPattern.solid(Color.kWhite);
    LEDPattern blinking = solidBlue.blink(Seconds.of(0.5)).atBrightness(Percent.of(10));
    Command blinkCommand = ledSubsystem.runPattern(blinking).ignoringDisable(true);

    private Optional<Alliance> ally; 
  
    public ShuffleboardTab fieldTab = Shuffleboard.getTab("Field"); 
    public final ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    public static ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

    //public DoubleSupplierSubscriber speedTunable = DogLog.tunable("Tunable Speed", () -> () -> Constants.Yeeter.YEETER_SPEED);
    //public DoubleSubscriber angleTunable = DogLog.tunable("Tunable Angle", Constants.Hood.HOOD_ANGLE);

    public RobotContainer() {
        configureBindings();
        //autoes.configDashboard(matchTab);
        theHood.configDashboard(matchTab);
        yeeter.configDashboard(matchTab);
        pivot.configDashboard(matchTab);
        configLLTab(limelightTab, fieldTab);
        climb.configDashboard(fieldTab);
        
        // Schedule the selected auto during the autonomous period
        // matchTab.add("auto chooser LOL", autoChooserLOL).withWidget(BuiltInWidgets.kComboBoxChooser);
        ally = DriverStation.getAlliance(); 
    }
    
    private void configureBindings() {
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

        /* brake mode */
        m_driverController.a().whileTrue(swerve.applyRequest(() -> brake));
        
        /* defense mode? */
        m_driverController.b().whileTrue(swerve.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        ));

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

        // reset the field-centric heading on left bumper press
        m_driverController.leftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));

        // hub alignment
        m_driverController.rightTrigger().whileTrue(
            swerve.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(limelight_aim_proportional()) // Drive with targetAngularVelocity
            )
        );

        m_driverController.leftTrigger().whileTrue(new AlignTowerPose(swerve));

        m_driverController.start().onTrue(swerve.runOnce(swerve::seedFieldCentric));

        // yaw setter --> 0 faces hub 
        // m_driverController.x().onTrue(new SequentialCommandGroup(
        //     // ROTATION2D IS IN **RADIANS!!!!**
        //     // SET YAW IS IN **DEGREES!!!!**
        //     new ConditionalCommand(
        //         new SequentialCommandGroup(
        //             new InstantCommand(() -> swerve.setOperatorPerspectiveForward(new Rotation2d(Math.PI))),
        //             new InstantCommand(() -> swerve.getPigeon2().setYaw(180.0)),
        //             new InstantCommand(() -> swerve.getPigeon2().getYaw().waitForUpdate(0.1)),
        //             new InstantCommand(() -> swerve.resetPose(new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(), new Rotation2d(Math.PI))))        
        //         ),
        //         new SequentialCommandGroup(
        //             new InstantCommand(() -> swerve.setOperatorPerspectiveForward(new Rotation2d(0.0))),    
        //             new InstantCommand(() -> swerve.getPigeon2().setYaw(0.0)),
        //             new InstantCommand(() -> swerve.getPigeon2().getYaw().waitForUpdate(0.1)),
        //             new InstantCommand(() -> swerve.resetPose(new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(), new Rotation2d(0))))
        //         ),
        //         () -> ally.get() == Alliance.Blue
        //     )
        // ));
    
        //MECHANISM DEFAULT COMMANDS
        //pivot.setDefaultCommand(new MovePivot(pivot, Constants.Pivot.SAFE));
        pivot.setDefaultCommand(new RunCommand(()-> pivot.maintainPosition(), pivot));
        yeeter.setDefaultCommand(new RunCommand(() -> yeeter.stopMotor(), yeeter));
        //theHood.setDefaultCommand(new RunninTheHood(theHood, Constants.Hood.HOOD_MIN));
        ledSubsystem.setDefaultCommand(ledSubsystem.runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));

        // SHOOTER AND HOOD BUTTON BINDINGS
        // m_operatorController.leftTrigger()
        // .whileTrue(
        //   //new SequentialCommandGroup(
        //     // new ParallelCommandGroup(
        //     //   new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED),
        //     //   new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED)
        //     // ),
        //     /*flywheelShooter.getNecessarySpeed(vision.getDisFromHub())*/

        //     //new SequentialCommandGroup(
        //       //new WaitUntilCommand(() -> flywheelShooter.reachedShooterSpeed()),
        //       new ParallelCommandGroup(
        //         new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED),
        //         new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED),
        //         new RunYeeter(yeeter, () -> yeeter.getNecessarySpeed(visabelle.getDisFromHub()), Constants.Yeeter.YEETER_ACCELERATION)
        //       )
        //     //)
        //   //)
        // );

        //SHOOTER AND HOOD BUTTON BINDINGS
        m_operatorController.leftTrigger().whileTrue(
           new SequentialCommandGroup( 
            new ParallelCommandGroup(
                new RunYeeter(yeeter, () -> Constants.Yeeter.YEETER_SPEED, Constants.Yeeter.YEETER_ACCELERATION), //() -> yeeter.getNecessarySpeed(() -> visabelle.getDisFromHub())

                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> yeeter.reachedYeeterSpeed()),
                    new RunFeeder(feeder, Constants.Feeder.PREP_FEEDER).withTimeout(0.5),
                    new ParallelCommandGroup(
                        new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED),
                        new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED)
                    ) 
                ),
                new MovePivot(pivot, Constants.Pivot.SLIGHTLY_UP_FROM_DOWN)
            ),
            new MovePivot(pivot, Constants.Pivot.DOWN_POSITION)
           )
        );
        //EJECT SHOOTER
        m_operatorController.leftTrigger().and(m_operatorController.x()).whileTrue(
                new RunYeeter(yeeter, ()-> -Constants.Yeeter.YEETER_SPEED, Constants.Yeeter.YEETER_ACCELERATION)
        );
        //SHUTTLING
        m_operatorController.leftBumper().whileTrue(
            new ParallelCommandGroup(
                new RunninTheHood(theHood, Constants.Hood.HOOD_ANGLE),
                new RunYeeter(yeeter, () -> Constants.Yeeter.YEETER_SPEED, Constants.Yeeter.YEETER_ACCELERATION)
            )
        );

        //m_operatorController.y().whileTrue(new RunHood(hood, Constants.Hood.HOOD_MAX));

        //PIVOT AND INTAKE AND INDEXER BUTTON BINDINGS
        //m_operatorController.leftTrigger().whileTrue(new MovePivot(pivot, Constants.Pivot.SAFE));
        
        //EJECT HOPPER
        m_operatorController.rightBumper().whileTrue(
            new RunFeeder(feeder, 5)
        );

        //INTAKE
        m_operatorController.rightTrigger().whileTrue(
          new ParallelCommandGroup(
            new MovePivot(pivot, Constants.Pivot.DOWN_POSITION), //wasnt there before
            new RunEater(eater, Constants.Eater.EATER_MOTOR_SPEED)
          )
        );
        //EJECT INTAKE
        m_operatorController.rightTrigger().and(m_operatorController.x()).whileTrue(
            new RunEater(eater, -Constants.Eater.EATER_MOTOR_SPEED)
          );
        //new RunIndexer(indexer, 10.0))); //is this formatting intended? why is feeder outside?

        m_operatorController.b().onTrue(new MovePivot(pivot, Constants.Pivot.DOWN_POSITION));
        m_operatorController.a().onTrue(new MovePivot(pivot, Constants.Pivot.SAFE));

        //CLIMB
        m_operatorController.y().onTrue(new MoveClimbHalfwayDown(climb, -4)); 
        m_operatorController.povUp().onTrue(new MoveClimbUp(climb, -15)); 
        m_operatorController.povDown().onTrue(new MoveClimbtoZero(climb, 15)); 

    }

    private double limelight_aim_proportional() {        
        double kP = 0.02; //0.035
        double targetingAngularVelocity = 0.0; 
        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.

        targetingAngularVelocity = (LimelightHelpers.getTX("limelight-front") * kP);

        // convert to radians per second for our drive method
        targetingAngularVelocity *= MaxAngularRate;
        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;
        return targetingAngularVelocity;
    }

    public void configLLTab(ShuffleboardTab tab, ShuffleboardTab fieldTab) {
        HttpCamera httpCamera1 = new HttpCamera("limelight-front", "http://10.19.67.14:5801/"); //http://10.19.67.202:5801/
        CameraServer.addCamera(httpCamera1);
        tab.add(httpCamera1).withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 0)
        .withSize(3, 2);

        HttpCamera httpCamera2 = new HttpCamera("limelight-back", "http://10.19.67.15:5801/"); //http://10.19.67.202:5801/
        CameraServer.addCamera(httpCamera2);
        tab.add(httpCamera2).withWidget(BuiltInWidgets.kCameraStream).withPosition(3, 0)
        .withSize(3, 2);

        tab.addBoolean("LL isInRange", () -> getInRange(LimelightHelpers.getTY("limelight-front")))
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(6, 1)
        .withSize(1, 1);

        tab.addBoolean("LL isAligned", () -> isAligned(LimelightHelpers.getTX("limelight-front")))
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(7, 1)
        .withSize(1, 1);

        //fieldTab.add("Field", CommandSwerveDrivetrain.m_field).withWidget(BuiltInWidgets.kField).withSize(8, 4);
    }

    public boolean getInRange(double position) {
        double target = 0.0;
        double threshold = 5.0;
        return position <=  (target + threshold) && position >= (target - threshold);
    }
  
    public boolean isAligned(double position) {
        double target = 0.0;
        double threshold = 5.0;
        return position <= (target + threshold) && position >= (target - threshold);
    }
}
