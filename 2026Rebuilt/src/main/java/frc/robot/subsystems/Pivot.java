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


public class Pivot extends SubsystemBase {
  private TalonFX pivotMotor;
  public double revsToMove;

  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor = new TalonFX (Constants.Pivot.PIVOT_MOTOR_ID);
    var talonFXconfigs = new TalonFXConfiguration();

    
    var slot0Configs = talonFXconfigs.Slot0;
    slot0Configs.kS = Constants.Pivot.kS; 
    slot0Configs.kV = Constants.Pivot.kV;
    slot0Configs.kA = Constants.Pivot.kA;
    slot0Configs.kP = Constants.Pivot.kP;
    slot0Configs.kI = Constants.Pivot.kI;
    slot0Configs.kD = Constants.Pivot.kD; 


    var motionMagicConfigs = talonFXconfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Pivot.CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Pivot.ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = Constants.Pivot.JERK;

    pivotMotor.getConfigurator().apply(talonFXconfigs);

      //resetEncoders(); 
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
      
  }

public void resetEncoders(){
  pivotMotor.setPosition(0);

}

public boolean isReached(){
  double rawPos = pivotMotor.getRotorPosition().getValueAsDouble();
  double currentPos = (rawPos/Constants.Pivot.GEAR_RATIO)*360;
  double diff = Math.abs(currentPos - revsToMove*360);
  return diff < 5.0;  
}

  

public void moveTo(double revolutions){
  revsToMove = revolutions*(Constants.Pivot.GEAR_RATIO);
  MotionMagicVoltage request = new MotionMagicVoltage(revsToMove);
}  

public void stopMotor(double speed){
  pivotMotor.stopMotor();
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

