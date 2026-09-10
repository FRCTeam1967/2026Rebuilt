// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;                             
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private TalonFX motor;
  private MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(0);

  /** Creates a new Feeder. */
  public Feeder() {
    motor = new TalonFX(Constants.Feeder.FEEDER_MOTOR_ID);

    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 40;
    limitConfigs.StatorCurrentLimitEnable = true;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = Constants.Feeder.kS;
    slot0Configs.kV = Constants.Feeder.kV;
    slot0Configs.kA = Constants.Feeder.kA;
    slot0Configs.kP = Constants.Feeder.kP;
    slot0Configs.kI = Constants.Feeder.kI;
    slot0Configs.kD = Constants.Feeder.kD;

    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Feeder.CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Feeder.ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = Constants.Feeder.JERK;

    motor.getConfigurator().apply(talonFXConfigs);
    motor.getConfigurator().apply(limitConfigs);
  }

  /**
   * @param speed - Sets motor to speed
   */
  public void setMotor(double speed){
    motor.set(speed);
  }

  /** 
   * @param speed Sets the motor to a velocity
  */
  public void setVelocity(double speed) {
    motor.setControl(motionMagicRequest.withVelocity(speed).withEnableFOC(true));
  }
  
  /**
   * @return If the feeder is stalling using the supply current
   */
  public boolean isStalling() {
    return (motor.getSupplyCurrent().getValueAsDouble() > 50.0); //65
  }

  /**
   * Stops motor
   */
  public void stopMotor(){
    motor.stopMotor();
  }  

  @Override
  public void periodic() {
    if (Constants.Feeder.verboseLogging) {
      DogLog.log("Feeder/stator current", motor.getStatorCurrent().getValueAsDouble());
    }
  }
}