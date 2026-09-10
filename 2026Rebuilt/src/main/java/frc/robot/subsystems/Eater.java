// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Eater extends SubsystemBase {
  private TalonFX motor;
  
  /** Creates a new Intake. */
  public Eater() {
    motor = new TalonFX(Constants.Eater.EATER_MOTOR_ID);
    var talonFXConfigurator = motor.getConfigurator();

    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 50;
    limitConfigs.StatorCurrentLimitEnable = true;

    var motorConfigs = new MotorOutputConfigs();

    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfigurator.apply(motorConfigs);
    talonFXConfigurator.apply(limitConfigs);
  }

  /**
   * @param speed - sets motor to speed
   */
  public void setMotor(double speed) {
    motor.set(speed);
  }
  /**
   * @return If the intake is stalling using the supply current
   */
  public boolean isStalling() {
    return (motor.getSupplyCurrent().getValueAsDouble() > 75.0); 
  }

  
  /**
   * Stops motor
   */
  public void stopMotor(){
    motor.stopMotor();
  }

  @Override
  public void periodic() {
  }
}