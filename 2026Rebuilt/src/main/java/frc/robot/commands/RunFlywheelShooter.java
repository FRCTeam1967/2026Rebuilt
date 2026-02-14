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


public class RunFlywheelShooter extends Command {
  /** Creates a new RunFlywheelShooter. */

  private final FlywheelShooter shooter;
  private final Feeder feeder;
  private final Indexer indexer;
  private final double speed1;
  private final double speed2;

  private boolean reachedShooterSpeed = false;
    
  public RunFlywheelShooter(FlywheelShooter shooter, Feeder feeder, Indexer indexer, double speed1, double speed2) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.indexer = indexer;
    this.speed1 = speed1;
    this.speed2 = speed2;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
  
  }

  @Override
  public void execute() {
    if (shooter.reachedShooterSpeed()) {
      shooter.setVelocity(speed2, speed1);
      feeder.setMotor(Constants.Feeder.FEEDER_SPEED);
      indexer.setMotor(Constants.Indexer.INDEXER_SPEED);
    }
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
