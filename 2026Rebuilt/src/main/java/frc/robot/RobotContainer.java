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

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climb;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.simulation.JoystickSim;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;  

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {    
    //drivetrain


    /* Setting up bindings for necessary control of the swerve drive platform */
    public final Climb climb = new Climb();

    //control
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    private Optional<Alliance> ally; 
  
    public ShuffleboardTab fieldTab = Shuffleboard.getTab("Field"); 
    public final ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    public static ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

    public RobotContainer() {
        configureBindings();
        climb.configDashboard(fieldTab);
        
        // Schedule the selected auto during the autonomous period
        // matchTab.add("auto chooser LOL", autoChooserLOL).withWidget(BuiltInWidgets.kComboBoxChooser);
        ally = DriverStation.getAlliance(); 

    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        // m_driverController.x().whileTrue(
        //     new ConditionalCommand(new RunCommand(() -> gerryRig.runMotor(0.7), gerryRig),
        //         new RunCommand(() -> gerryRig.stopMotor(), gerryRig), 
        //         () -> autoes.getDisSensor() <= 0.15)
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        // hub alignment

        //m_driverController.leftTrigger().whileTrue(new AlignTowerPose(drivetrain));
    

        //SHOOTER AND HOOD BUTTON BINDINGS
        m_operatorController.leftTrigger()
        .whileTrue(
          new SequentialCommandGroup(
            // new ParallelCommandGroup(
            //   new RunIndexer(indexer, Constants.Indexer.INDEXER_SPEED),
            //   new RunFeeder(feeder, Constants.Feeder.FEEDER_SPEED)
            // ),
            /*flywheelShooter.getNecessarySpeed(vision.getDisFromHub())*/
            //)
          )
        );

        //m_operatorController.y().whileTrue(new RunHood(hood, Constants.Hood.HOOD_MAX));

        //PIVOT AND INTAKE AND INDEXER BUTTON BINDINGS
        //m_operatorController.leftTrigger().whileTrue(new MovePivot(pivot, Constants.Pivot.SAFE));
        
        // eject button

        //CLIMB
        m_operatorController.y().onTrue(new MoveClimbHalfwayDown(climb, -4)); 
        m_operatorController.povUp().onTrue(new MoveClimbUp(climb, -15)); 
        m_operatorController.povDown().onTrue(new MoveClimbtoZero(climb, 15)); 

    }
}
