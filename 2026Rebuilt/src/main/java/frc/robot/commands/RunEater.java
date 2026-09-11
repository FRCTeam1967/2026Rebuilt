// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Eater;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunEater extends Command {
  public Eater eater;
  private double speed;

  /** Creates a new RunEater. 
   * @param eater - Eater (intake) subsystem
   * @param speed - Requested speed for the eater
  */
  public RunEater(Eater eater, double speed) {
    this.eater = eater;
    this.speed = speed;
    addRequirements(eater);
  }

  /**Called when the command is initially scheduled.*/
  @Override
  public void initialize() {}

  /**Called every time the scheduler runs and sets the motor to a requested speed*/
  @Override
  public void execute() {
    eater.setMotor(speed);
  }

  /**Called once the command ends or is interrupted and stops the motor.*/
  @Override
  public void end(boolean interrupted) {
    eater.stopMotor();
  }

  /** 
   * @return true when the command is finished 
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
