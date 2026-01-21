// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveHood extends Command {
  private Hood hood;
  private double hoodVelocity, hoodAcceleration;

  /** Creates a new MoveHood. */
  public MoveHood(Hood hood, double hoodVelocity, double hoodAcceleration) {
    this.hood = hood;
    this.hoodVelocity = hoodVelocity;
    this.hoodAcceleration = hoodAcceleration;
    //addRequirements(hood, hoodVelocity, hoodAcceleration);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  //@Override
  // public void execute() {
  //   hood.moveTo(hood.findAngle(tz, hoodVelocity)); //tz will be put in later for vision

  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hood.isReached();
  }
}
