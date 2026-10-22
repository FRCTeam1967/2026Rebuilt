// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.InvertedValue;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Eater extends SubsystemBase {
  private TalonFX motor;
  private final CANBus canbus = RobotContainer.CANBus;
  
  /** Creates a new Intake. */
  public Eater() {
    motor = new TalonFX(Constants.Eater.EATER_MOTOR_ID, canbus);


    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 75;
    limitConfigs.StatorCurrentLimitEnable = true;

    var motorConfigs = new MotorOutputConfigs();

    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(motorConfigs);
    motor.getConfigurator().apply(limitConfigs);
  }

  /**
   * @param speed - sets motor to speed
   */
  public void setMotor(double speed) {
    VelocityVoltage request = new VelocityVoltage(speed);
    motor.setControl(request);
  }

  public boolean isStalling() {
    return (motor.getSupplyCurrent().getValueAsDouble() > 50.0); //65
  }
  
  /**
   * stops motor
   */
  public void stopMotor(){
    motor.stopMotor();
  }

  public double getVelocityRPS(){
    return motor.getRotorVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("eater speed", motor.getRotorVelocity().getValueAsDouble());
    //SmartDashboard.putNumber("eater current draw", motor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("eater speed", motor.getRotorVelocity().getValueAsDouble());
    Logger.recordOutput("eater current draw", motor.getSupplyCurrent().getValueAsDouble());
  }
}