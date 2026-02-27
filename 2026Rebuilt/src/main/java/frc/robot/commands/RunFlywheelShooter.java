// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.FlywheelShooter;
import frc.robot.commands.RunIndexer;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Feeder;
import java.util.function.DoubleSupplier;
import dev.doglog.DogLog;



public class RunFlywheelShooter extends Command {
  /** Creates a new RunFlywheelShooter. */

  private final FlywheelShooter shooter;
  private final double speed;
  private final double acceleration;
    
  public RunFlywheelShooter(FlywheelShooter shooter, double speed, double acceleration) {
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
    shooter.stopMotor();
  }

 // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
