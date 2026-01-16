// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  
  private TalonFX motor; 
  private TalonFXConfiguration config;
  private MotionMagicVoltage request;
  private boolean isRight;
  private DigitalInput sensor;

  public Climb(int motorID){
    motor = new TalonFX(motorID);
    config = new TalonFXConfiguration();
    request = new MotionMagicVoltage(0);

    config.Slot0.kP = Constants.Climb.UP_kP;
    config.Slot0.kI = Constants.Climb.UP_kI;
    config.Slot0.kD = Constants.Climb.UP_kD;
    config.Slot0.kS = Constants.Climb.UP_kS;

    config.Slot1.kP = Constants.Climb.DOWN_kP;
    config.Slot1.kI = Constants.Climb.DOWN_kI;
    config.Slot1.kD = Constants.Climb.DOWN_kD;
    config.Slot1.kS = Constants.Climb.DOWN_kS;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Climb.CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = Constants.Climb.ACCELERATION;
    
    config.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Climb.CURRENT_LIMIT));
    
    motor.getConfigurator().apply(config);

  }

  public void setEncoderOffset() {
     motor.setPosition(0);
  }

  public void moveTo(double pos, boolean holdingRobot) {
    if (holdingRobot) {
      motor.setControl(request.withPosition(pos).withSlot(1));
    } else {
      motor.setControl(request.withPosition(pos).withSlot(0));
    }
  }

   public void moveAt(DoubleSupplier speed) {
    if(speed.getAsDouble() < 0) {
      motor.set(speed.getAsDouble() * Constants.Climb.WIND_FACTOR);
    } else {
      motor.set(speed.getAsDouble() * Constants.Climb.UNWIND_FACTOR);
    }
  }

   public void lowerAt(double speed){
    motor.set(speed * Constants.Climb.WIND_FACTOR);
  }

   public void stop() {
    motor.stopMotor();
  }

  public boolean getSensorValue() {
    return sensor.get();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

