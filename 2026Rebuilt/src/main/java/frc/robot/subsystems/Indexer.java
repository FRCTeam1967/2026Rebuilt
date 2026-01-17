// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private TalonFX motor;
  /** Creates a new Indexer. */
  public Indexer() {
    motor = new TalonFX(Constants.Indexer.INDEXER_MOTOR_ID);
  }
  public void setMotor(double speed){
    motor.set(speed);
  }
  public void stopMotor(){
    motor.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
