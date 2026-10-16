// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimHub extends Command {

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final RobotContainer m_robotContainer; 
  private Visabelle visabelle;

  private final SwerveRequest.FieldCentricFacingAngle driveAtAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1) // 0.1 = deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    
  public AimHub(RobotContainer m_robotContainer, Visabelle visabelle) {
    this.m_robotContainer = m_robotContainer;
    this.visabelle = m_robotContainer.visabelle;

    addRequirements(m_robotContainer.swerve);
  }
  
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotContainer.swerve.setControl(
      driveAtAngle
        .withTargetDirection(new Rotation2d(visabelle.getAngleToHub()))
        .withVelocityX(0)
        .withVelocityY(0)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
