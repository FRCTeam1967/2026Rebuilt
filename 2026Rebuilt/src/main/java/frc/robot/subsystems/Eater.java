// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Eater extends SubsystemBase {
  private TalonFX motor;
  private final CANBus canbus = RobotContainer.CANBus;
  private final DoubleSubscriber intakeSpeed = DogLog.tunable("Eater/intakeSpeed", Constants.Eater.EATER_MOTOR_SPEED);
  private MotionMagicVoltage request;
  private double rotations;

  /** Creates a new Intake. */
  public Eater() {
    motor = new TalonFX(Constants.Eater.EATER_MOTOR_ID);
    var talonFXConfigurator = motor.getConfigurator();

    var limitConfigs = new CurrentLimitsConfigs();
  
    var motorConfigs = new MotorOutputConfigs();

    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kS = Constants.Feeder.kS;
    slot0Configs.kV = Constants.Feeder.kV;
    slot0Configs.kA = Constants.Feeder.kA;
    slot0Configs.kP = Constants.Feeder.kP;
    slot0Configs.kI = Constants.Feeder.kI;
    slot0Configs.kD = Constants.Feeder.kD;
    


    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    //var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motor.getConfigurator().apply(talonFXConfigs);
    talonFXConfigurator.apply(motorConfigs);
    talonFXConfigurator.apply(limitConfigs);

  }

  /**
   * @param speed - sets motor to speed
   */  
  public void setMotor(double speed) {
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(speed);
    motor.setControl(request);
    DogLog.log("Eater/intake desired speed", intakeSpeed.get());

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
    if (Constants.Eater.verboseLogging) {
      DogLog.log("Eater/stator current", motor.getStatorCurrent().getValueAsDouble());
    }
  }
}

