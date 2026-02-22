// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.VisionUpdate;
import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    public LED ledSubsystem = new LED();
    LEDPattern solidBlue = LEDPattern.solid(Color.kWhite);
    LEDPattern blinking = solidBlue.blink(Seconds.of(0.5)).atBrightness(Percent.of(10));
    Command blinkCommand = ledSubsystem.runPattern(blinking).ignoringDisable(true);

    //public Autoes autoes;
    public final Pivot pivot = new Pivot();
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer();
    public final Feeder feeder = new Feeder();

    public ShuffleboardTab fieldTab = Shuffleboard.getTab("Field");
    
    public final FlywheelShooter flywheelShooter = new FlywheelShooter();
    //private final Hood hood = new Hood();  
    public final ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
        
    public VisionUpdate visionUpdate = new VisionUpdate(drivetrain);
    
    public boolean isBlue;
    Optional<Alliance> ally = DriverStation.getAlliance(); 

    public Vision vision = new Vision(drivetrain, MaxAngularRate);

    public RobotContainer() {  
        //autoes = new Autoes(this);
        configureBindings();
        //autoes.configDashboard(matchTab);
        //hood.configDashboard(matchTab);
        flywheelShooter.configDashboard(matchTab);
        pivot.configDashboard(fieldTab);
        configLLTab(limelightTab, fieldTab);   
      
        flywheelShooter.setDefaultCommand(
            new RunCommand(() -> flywheelShooter.stopMotor(), flywheelShooter)
        );
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
        //reset gyro
        m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

        m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
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
                    .withRotationalRate(vision.limelight_aim_proportional()) // Drive with targetAngularVelocity
            )
        );
        // // ranging
        // joystick.rightTrigger().whileTrue(
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(limelight_range_proportional()) // Drive forward with negative Y (forward)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // tower alignment using TX
        //joystick.leftTrigger().whileTrue(new AlignTowerTX(drivetrain, vision, false, true));

        // tower alignment using pose
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
            
        drivetrain.registerTelemetry(logger::telemeterize);
      //SHOOTER AND HOOD BUTTON BINDINGS
      m_operatorController.x()
      .whileTrue(
        new SequentialCommandGroup(
          // new ParallelCommandGroup(
          //   new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED),
          //   new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED)
          // ),
          new ParallelRaceGroup(
            new RunFlywheelShooter(flywheelShooter, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED, Constants.FlywheelShooter.FLYWHEEL_SHOOTER_ACCELERATION),
            new WaitCommand(5.0)
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

      // eject button
      m_operatorController.rightBumper().whileTrue(
        new RunFeeder(feeder, 5)
      );


      //m_operatorController.y().onTrue(new RunHood(hood, Constants.Hood.HOOD_MAX));

      //INTAKE AND INDEXER BUTTON BINDINGS
      m_operatorController.leftTrigger().whileTrue(new MovePivot(pivot, Constants.Pivot.SAFE));

      m_operatorController.rightTrigger().whileTrue(
        new ParallelCommandGroup(
          new MovePivot(pivot, Constants.Pivot.DOWN_POSITION), //wasnt there before
          new RunIntake(intake, Constants.Intake.INTAKE_MOTOR_SPEED),
          new RunIndexer(indexer, 10.0))); //is this formatting intended?
          new RunFeeder(feeder, -Constants.Feeder.FEEDER_SPEED);

      m_operatorController.b().onTrue(new MovePivot(pivot, Constants.Pivot.DOWN_POSITION));

      m_operatorController.a().onTrue(new MovePivot(pivot, Constants.Pivot.SAFE));

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
