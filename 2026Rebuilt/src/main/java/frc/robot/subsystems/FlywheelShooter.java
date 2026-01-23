// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlywheelShooter extends SubsystemBase {

  private TalonFX flywheelMotor;

  /** Creates a new FlywheelShooter. */
  public FlywheelShooter() {
    flywheelMotor = new TalonFX(Constants.FlywheelShooter.FLYWHEELSHOOTER_MOTOR_ID);

      var talonFXConfigs = new TalonFXConfiguration();

      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = Constants.FlywheelShooter.kS; 
      slot0Configs.kV = Constants.FlywheelShooter.kV; 
      slot0Configs.kA = Constants.FlywheelShooter.kA; 
      slot0Configs.kP = Constants.FlywheelShooter.kP; 
      slot0Configs.kI = Constants.FlywheelShooter.kI;
      slot0Configs.kD = Constants.FlywheelShooter.kD; 
  
      // set Motion Magic settings
      var motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.FlywheelShooter.CRUISE_VELOCITY;
      motionMagicConfigs.MotionMagicAcceleration = Constants.FlywheelShooter.ACCELERATION;
      motionMagicConfigs.MotionMagicJerk = Constants.FlywheelShooter.JERK;

      talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

      flywheelMotor.setNeutralMode(NeutralModeValue.Brake);
      //talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      flywheelMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void setMotor(double speed) {
      flywheelMotor.set(speed);
  }

  //public void moveTo(double revolutions) {
      //MotionMagicVoltage request = (new MotionMagicVoltage(revolutions)).withFeedForward(0.0);
      //flywheelMotor.setControl(request);
  //}

  public void setVelocity(double velocity) {
      //MotionMagicVelocityDutyCycle request = new MotionMagicVelocityDutyCycle(velocity);
      MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(velocity);

      flywheelMotor.setControl(request);
  }

  //public void moveMotor(double speed) {
      //VelocityVoltage request = new VelocityVoltage(speed);
      //intakeMotor.setControl(request);
   //}
  
   /** Stops both the left motor and right motor */
   public void stopMotor() {
      flywheelMotor.stopMotor();
   }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
