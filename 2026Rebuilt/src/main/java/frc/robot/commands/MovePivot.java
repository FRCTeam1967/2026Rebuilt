// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MovePivot extends Command {
  /** Creates a new MovePivot. */
  private Pivot pivot;
  private double targetPosition;
  private boolean isSlow;

  /**
   * Convenience initalizer. Moves pivot quickly. This alleviates the need
   * to change all call sites to pass a 3rd parameter when they want the default
   * behavior that used to exist.
   * @param pivot Pivot subsystem
   * @param targetPosition desired position (rotations)
   */
  public MovePivot(Pivot pivot, double targetPosition) {
    this(pivot, targetPosition, false);
  }
  
  public MovePivot(Pivot pivot, double targetPosition, boolean isSlow) {
    this.pivot = pivot;
    this.targetPosition = targetPosition;
    this.isSlow = isSlow;
    addRequirements(this.pivot);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.moveTo(targetPosition, isSlow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivot.isReached();
  }
}