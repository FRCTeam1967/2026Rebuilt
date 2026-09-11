// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MovePivot extends Command {
  private Pivot pivot;
  private double targetPosition;
  private boolean isSlow;

  /** Creates a new MovePivot. */
  public MovePivot(Pivot pivot, double targetPosition) {
    this(pivot, targetPosition, false);
  }
  
  /** Creates a new MovePivot. 
   * @param pivot - Pivot (intake pivot) subsystem
   * @param targetPosition - Requested position for the pivot in revolutions
   * @param isSlow - Returns if movement is or is not slow
  */
  public MovePivot(Pivot pivot, double targetPosition, boolean isSlow) {
    this.pivot = pivot;
    this.targetPosition = targetPosition;
    this.isSlow = isSlow;
    addRequirements(this.pivot);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {}

  /** Called every time the scheduler runs while the command is scheduled and moves the pivot to a requested position, and may be slow depending on boolean value */
  @Override
  public void execute() {
     pivot.moveTo(targetPosition, isSlow);
  }

  /** Called once the command ends or is interrupted. */
  @Override
  public void end(boolean interrupted) {}

  /** 
   * @return true when the pivot has reached its target position
  */
  @Override
  public boolean isFinished() {
    return pivot.isReached();
  }
}