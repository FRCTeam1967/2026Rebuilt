// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import java.util.Optional;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public LED ledSubsystem = new LED();
    public Autoes autoes = new Autoes(this);
    public final Pivot pivot = new Pivot();
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer();
    public final Feeder feeder = new Feeder();
    public final FlywheelShooter flywheelShooter = new FlywheelShooter();
    public final Hood hood = new Hood();
    public VisionUpdate visionUpdate = new VisionUpdate(drivetrain);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    LEDPattern solidBlue = LEDPattern.solid(Color.kWhite);
    LEDPattern blinking = solidBlue.blink(Seconds.of(0.5)).atBrightness(Percent.of(10));
    Command blinkCommand = ledSubsystem.runPattern(blinking).ignoringDisable(true);

    public ShuffleboardTab fieldTab = Shuffleboard.getTab("Field"); 
    public final ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    public static ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    private Optional<Alliance> ally;

    public RobotContainer() {
        configureBindings();
        autoes.configDashboard(matchTab);
        hood.configDashboard(matchTab);
        flywheelShooter.configDashboard(matchTab);
        pivot.configDashboard(fieldTab);
        configLLTab(limelightTab, fieldTab);   
        
        // Schedule the selected auto during the autonomous period
        // matchTab.add("auto chooser LOL", autoChooserLOL).withWidget(BuiltInWidgets.kComboBoxChooser);
        ally = DriverStation.getAlliance(); 

    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        /* reset gyro */
        m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));//.seedFieldCentric()

        //POV buttons
        m_driverController.povUp().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(0.2 * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        m_driverController.povDown().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-0.2 * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        m_driverController.povRight().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-0.2 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        m_driverController.povLeft().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(0.2 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        /* brake mode */
        m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        /* defense mode? */
        m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        ));

        // m_driverController.x().whileTrue(
        //     new ConditionalCommand(new RunCommand(() -> gerryRig.runMotor(0.7), gerryRig),
        //         new RunCommand(() -> gerryRig.stopMotor(), gerryRig), 
        //         () -> autoes.getDisSensor() <= 0.15)
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driverController.povDown().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.povDown().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.povUp().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.povUp().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // hub alignment
        m_driverController.rightTrigger().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(limelight_aim_proportional()) // Drive with targetAngularVelocity
            )
        );

        m_driverController.leftTrigger().whileTrue(new AlignTowerPose(drivetrain));

        m_driverController.x().onTrue(new SequentialCommandGroup(
            // ROTATION2D IS IN **RADIANS!!!!**
            // SET YAW IS IN **DEGREES!!!!**
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> drivetrain.setOperatorPerspectiveForward(new Rotation2d(Math.PI))),
                    new InstantCommand(() -> drivetrain.getPigeon2().setYaw(180.0)),
                    new InstantCommand(() -> drivetrain.getPigeon2().getYaw().waitForUpdate(0.1)),
                    new InstantCommand(() -> drivetrain.resetPose(new Pose2d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), new Rotation2d(Math.PI))))        
                ),
                new SequentialCommandGroup(
                    new InstantCommand(() -> drivetrain.setOperatorPerspectiveForward(new Rotation2d(0.0))),    
                    new InstantCommand(() -> drivetrain.getPigeon2().setYaw(0.0)),
                    new InstantCommand(() -> drivetrain.getPigeon2().getYaw().waitForUpdate(0.1)),
                    new InstantCommand(() -> drivetrain.resetPose(new Pose2d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), new Rotation2d(0))))
                ),
                () -> ally.get() == Alliance.Blue
            )
        ));
    
      //MECHANISM DEFAULT COMMANDS
      //pivot.setDefaultCommand(new MovePivot(pivot, Constants.Pivot.SAFE));
      pivot.setDefaultCommand(new RunCommand(()-> pivot.maintainPosition(), pivot));
      flywheelShooter.setDefaultCommand(new RunCommand(() -> flywheelShooter.stopMotor(), flywheelShooter));
      hood.setDefaultCommand(new RunCommand(()-> hood.maintainPosition(), hood));

      //SHOOTER AND HOOD BUTTON BINDINGS
      m_operatorController.leftTrigger()
      .whileTrue(
        new SequentialCommandGroup(
          // new ParallelCommandGroup(
          //   new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED),
          //   new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED)
          // ),
          new ParallelRaceGroup(
            new RunFlywheelShooter(flywheelShooter, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
            new WaitCommand(2.5)
          ),
          //new SequentialCommandGroup(
            //new WaitUntilCommand(() -> flywheelShooter.reachedShooterSpeed()),
            new ParallelCommandGroup(
              new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED),
              new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED),
              new RunFlywheelShooter(flywheelShooter, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION)
            )
          //)
        )
      );

      m_operatorController.y()
      .onTrue(new RunHood(hood, Constants.Hood.HOOD_MAX));

      //PIVOT AND INTAKE AND INDEXER BUTTON BINDINGS
      //m_operatorController.leftTrigger().whileTrue(new MovePivot(pivot, Constants.Pivot.SAFE));
      // eject button
      m_operatorController.rightBumper().whileTrue(
        new RunFeeder(feeder, 5)
      );

      m_operatorController.rightTrigger().whileTrue(
        new ParallelCommandGroup(
          new MovePivot(pivot, Constants.Pivot.DOWN_POSITION), //wasnt there before
          new RunIntake(intake, Constants.Intake.INTAKE_MOTOR_SPEED)));
          //new RunIndexer(indexer, 10.0))); //is this formatting intended? why is feeder outside?
          
          new RunFeeder(feeder, -Constants.Feeder.FEEDER_SPEED);

      m_operatorController.b().onTrue(new MovePivot(pivot, Constants.Pivot.DOWN_POSITION));

      m_operatorController.a().onTrue(new MovePivot(pivot, Constants.Pivot.SAFE));

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

    // public Command getAutonomousCommand() {
    //     return Commands.print("No autonomous command configured");
    // }

    public void configLLTab(ShuffleboardTab tab, ShuffleboardTab fieldTab) {
        HttpCamera httpCamera1 = new HttpCamera("limelight-front", "http://10.19.67.13:5801/"); //http://10.19.67.202:5801/
        CameraServer.addCamera(httpCamera1);
        tab.add(httpCamera1).withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 0)
        .withSize(3, 2);

        HttpCamera httpCamera2 = new HttpCamera("limelight-back", "http://10.19.67.11:5801/"); //http://10.19.67.202:5801/
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
        return position <=  (target + threshold) && position >= (target - threshold);
    }
}
