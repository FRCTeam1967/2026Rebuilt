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

  private final FlywheelShooter shooter1;
  private final FlywheelShooter shooter2;
  private final FlywheelShooter shooter3;
  private final double speed;
    
  public RunFlywheelShooter(FlywheelShooter shooter1, FlywheelShooter shooter2, FlywheelShooter shooter3, double speed) {
    this.shooter1 = shooter1;
    this.shooter2 = shooter2;
    this.shooter3 = shooter3;
    this.speed = speed;

    addRequirements(shooter1);
    addRequirements(shooter2);
    addRequirements(shooter3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

   // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter1.setMotor(speed);
    shooter2.setMotor(speed);
    shooter3.setMotor(speed);
  }
  
   @Override
  public void end(boolean interrupted) {
    shooter1.stopMotor();
    shooter2.stopMotor();
    shooter3.stopMotor();
  }

 // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return intake.intakeBroken(); 
    return false;
  }
}
