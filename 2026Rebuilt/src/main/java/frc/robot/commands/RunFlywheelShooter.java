// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.FlywheelShooter;
import java.util.function.DoubleSupplier;

public class RunFlywheelShooter extends Command {
  /** Creates a new RunFlywheelShooter. */

  private final FlywheelShooter shooter;
  private final double speed;
    
  public RunFlywheelShooter(FlywheelShooter shooter, double speed) {
    this.shooter = shooter;
    this.speed = speed;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
  
  }

  @Override
  public void execute() {
    shooter.setVelocity(speed);
  }
  
   @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
  }

 // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
