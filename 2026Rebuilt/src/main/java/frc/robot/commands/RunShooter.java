// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooter;
import java.util.function.DoubleSupplier;



public class RunShooter extends Command {
  /** Creates a new RunFlywheelShooter. */
  private final IntakeShooter shooter;
  private final DoubleSupplier speed;
  private final double acceleration;

  public RunShooter(IntakeShooter shooter, DoubleSupplier speed, double acceleration) {
    this.shooter = shooter;
    this.speed = speed;
    this.acceleration = acceleration;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
  
  }

  @Override
  public void execute() {
    shooter.setVelocity(speed, acceleration);
  }
  
   @Override
  public void end(boolean interrupted) {
    //shooter.stopMotor();
  }

 // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
