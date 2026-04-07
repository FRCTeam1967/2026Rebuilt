// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private TalonFX motor;
  private final CANBus canbus = RobotContainer.CANBus;
  private MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(0);

  /** Creates a new Indexer. */
  public Indexer() {
    motor = new TalonFX(Constants.Indexer.INDEXER_MOTOR_ID);

    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = Constants.Indexer.kS;
    slot0Configs.kV = Constants.Indexer.kV;
    slot0Configs.kA = Constants.Indexer.kA;
    slot0Configs.kP = Constants.Indexer.kP;
    slot0Configs.kI = Constants.Indexer.kI;
    slot0Configs.kD = Constants.Indexer.kD;

    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Indexer.CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Indexer.ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = Constants.Indexer.JERK;
    
    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 10;
    limitConfigs.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(talonFXConfigs);
  }

  /**
   * @param speed - sets motor to speed
   */
  public void setMotor(double speed){
    motor.set(speed);
      DogLog.log("indexer desired speed", speed);
  }

  public void setVelocity(double speed) {
    motor.setControl(motionMagicRequest.withVelocity(speed));
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
    if (Constants.Indexer.verboseLogging) {
      DogLog.log("Indexer/stator current", motor.getStatorCurrent().getValueAsDouble());
    }
  }
}