// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import org.wpilib.vision.stream.CameraServer;
import org.wpilib.vision.camera.HttpCamera;
import org.wpilib.math.geometry.Pose2d;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.driverstation.DriverStation;

import frc.robot.commands.*;

import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.command2.button.Trigger;
import org.wpilib.command2.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.*;
import org.wpilib.command2.*;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    //drivetrain

    //mechanism
    public static final CANBus CANBus = new CANBus("Default Name");
    public final Eater eater = new Eater();

    //control
    private final CommandGamepad m_operatorController = new CommandGamepad(0);


    public RobotContainer() {
        configureBindings();
        //theHood.configDashboard(matchTab);
        //yeeter.configDashboard(matchTab);
        //pivot.configDashboard(matchTab);
        //configLLTab(limelightTab, fieldTab);
        //climb.configDashboard(fieldTab);
        
        // Schedule the selected auto during the autonomous period
        // matchTab.add("auto chooser LOL", autoChooserLOL).withWidget(BuiltInWidgets.kComboBoxChooser);
    }
    

    private void configureBindings() {
        m_operatorController.northFace().onTrue(new RunEater(eater, Constants.Eater.EATER_MOTOR_SPEED));
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
