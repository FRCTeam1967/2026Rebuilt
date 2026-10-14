// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import java.util.Optional;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.PowerDistribution;


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
    private final CommandXboxController m_operatorController = new CommandXboxController(0);


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
        m_operatorController.y().onTrue(new RunEater(eater, Constants.Eater.EATER_MOTOR_SPEED));
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
