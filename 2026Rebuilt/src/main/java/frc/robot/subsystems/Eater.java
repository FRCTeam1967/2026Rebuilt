// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Eater extends SubsystemBase {
  private TalonFX motor;
  private final CANBus canbus = RobotContainer.CANBus;
  
  /** Creates a new Intake. */
  public Eater() {
    motor = new TalonFX(Constants.Eater.EATER_MOTOR_ID, canbus);
    var talonFXConfigurator = motor.getConfigurator();
    var motorConfigs = new MotorOutputConfigs();

    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfigurator.apply(motorConfigs);
  }

  /**
   * @param speed - sets motor to speed
   */
  public void setMotor(double speed) {
    motor.set(speed);
  }
  
  /**
   * stops motor
   */
  public void stopMotor(){
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

