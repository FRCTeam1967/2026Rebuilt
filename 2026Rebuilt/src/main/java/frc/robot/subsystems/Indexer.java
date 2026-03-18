// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private TalonFX motor;
  private final CANBus canbus = RobotContainer.CANBus;

  /** Creates a new Indexer. */
  public Indexer() {
    motor = new TalonFX(Constants.Indexer.INDEXER_MOTOR_ID, canbus);

    var talonFXConfigurator = motor.getConfigurator();
    var motorConfigs = new MotorOutputConfigs();

    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 40;
    limitConfigs.StatorCurrentLimitEnable = true;

    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfigurator.apply(motorConfigs);
  }

  /**
   * @param speed - sets motor to speed
   */
  public void setMotor(double speed){
    motor.set(speed);
    DogLog.log("indexer desired speed", speed);
  }

  /**
   * stops motor
   */
  public void stopMotor(){
    motor.stopMotor();
  }

  public void logVoltage() {
    DogLog.log("indexer voltage", motor.getStatorCurrent().getValueAsDouble());
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
