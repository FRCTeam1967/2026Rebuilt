// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopingArm;
import frc.robot.Constants;

public class ManualArm extends Command {
  private TelescopingArm telescopingArm;
  private DoubleSupplier speed;
  
  public ManualArm(DoubleSupplier speed, TelescopingArm telescopingArm) {
    this.telescopingArm = telescopingArm;
    this.speed = speed;
    addRequirements(telescopingArm);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    if(telescopingArm.isEnabled() && !telescopingArm.getSensorValue() && telescopingArm.getEncoderCount() > Constants.TelescopingArm.TOP_ROTATIONS){
      telescopingArm.runMotor(speed);
    } else if(telescopingArm.isEnabled() && telescopingArm.getSensorValue() && speed.getAsDouble() < 0){
      telescopingArm.runMotor(speed);
    } else {
      telescopingArm.stop();
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    telescopingArm.stop();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}