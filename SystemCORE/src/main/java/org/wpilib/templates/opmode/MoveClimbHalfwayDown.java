// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.templates.opmode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveClimbHalfwayDown extends Command {
  private Climb climb;
  private double inches;
  
  /** Creates a new MoveClimbHalfwayDown. */
  public MoveClimbHalfwayDown(Climb climb, double inches) {
    this.climb = climb;
    this.inches = inches;
    addRequirements(climb);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climb.atHeight();
  }
}
