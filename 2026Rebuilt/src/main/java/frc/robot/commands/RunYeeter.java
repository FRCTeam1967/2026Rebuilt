// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Yeeter;
import java.util.function.DoubleSupplier;


public class RunYeeter extends Command {
  private final Yeeter yeeter;
  private final DoubleSupplier speed;
  private final double acceleration;

  /** Creates a new RunFlywheelShooter.
   * @param yeeter Yeeter (shooter) subsystem
   * @param speed Requested speed for the yeeter
   * @param acceleration Requested acceleration for the yeeter
  */
  public RunYeeter(Yeeter yeeter, DoubleSupplier speed, double acceleration) {
    this.yeeter = yeeter;
    this.speed = speed;
    this.acceleration = acceleration;
    addRequirements(yeeter);
  }

  /**Called when the command is initially scheduled.*/
  @Override
  public void initialize() {}

  /**Called every time to update the shooter's target speed and acceleration*/
  @Override
  public void execute() {
    yeeter.setVelocity(speed, acceleration);
  }
  
  /**Called once the command ends or is interrupted.*/
   @Override
  public void end(boolean interrupted) {}

  /**
   * @return true when the command is finished
  */
  @Override
  public boolean isFinished() {
    return false;
  }
}
