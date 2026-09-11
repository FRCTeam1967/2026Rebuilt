// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveClimbUp extends Command {
  private Climb climb;
  private double inches;

  /** Creates a new MoveClimb. 
   * @param climb - Climb subsystem
   * @param inches - Requested height in inches for the climb
  */
  public MoveClimbUp(Climb climb, double inches) {
    this.climb = climb;
    this.inches = inches;
    addRequirements(climb);
  }

  /**Called when the command is initially scheduled.*/
  @Override
  public void initialize(){}

  /** Called every time the scheduler runs while the command is scheduled and moves the climb to the requested height. */
  @Override
  public void execute() {
    climb.moveTo(inches);
  }

  /** Called once the command ends or is interrupted and stops the motor. */
  @Override
  public void end(boolean interrupted) {
      climb.stopMotor();
  }

  /** 
   * @return true when the climb has reached its fully up target position
  */
  @Override
  public boolean isFinished() {
    return (climb.isReachedTopSwitch());
  }
}



