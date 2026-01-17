// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants;


public class Extension extends SubsystemBase {
  private TalonFX motor;
  public double revsToMove;

  /** Creates a new Pivot. */
  public Extension() {
    motor = new TalonFX (Constants.Extension.EXTENSION_MOTOR_ID);
    var talonFXconfigs = new TalonFXConfiguration();

    
    var slot0Configs = talonFXconfigs.Slot0;
    slot0Configs.kS = Constants.Extension.kS; 
    slot0Configs.kV = Constants.Extension.kV;
    slot0Configs.kA = Constants.Extension.kA;
    slot0Configs.kP = Constants.Extension.kP;
    slot0Configs.kI = Constants.Extension.kI;
    slot0Configs.kD = Constants.Extension.kD; 


    var motionMagicConfigs = talonFXconfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Extension.CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Extension.ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = Constants.Extension.JERK;

    motor.getConfigurator().apply(talonFXconfigs);

      //resetEncoders(); 
    motor.setNeutralMode(NeutralModeValue.Brake);
      
  }

public void resetEncoders(){
  motor.setPosition(0);

}

public boolean isReached(){
  double rawPos = motor.getRotorPosition().getValueAsDouble();
  double currentPos = (rawPos/Constants.Extension.GEAR_RATIO)*360;
  double diff = Math.abs(currentPos - revsToMove*360);
  return diff < 5.0;  
}

  

public void moveTo(double revolutions){
  revsToMove = revolutions*(Constants.Extension.GEAR_RATIO);
  MotionMagicVoltage request = new MotionMagicVoltage(revsToMove);
}  

public void stopMotor(double speed){
  motor.stopMotor();
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

