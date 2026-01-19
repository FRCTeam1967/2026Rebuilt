// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class VelocityControl extends SubsystemBase {
  private TalonFX motor1;
  /** Creates a new VelocityControl. */
  public VelocityControl() {



    var talonFXConfigs = new TalonFXConfiguration();


      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = Constants.VelocityControl.kS; 
      slot0Configs.kV = Constants.VelocityControl.kV; 
      slot0Configs.kA = Constants.VelocityControl.kA; 
      slot0Configs.kP = Constants.VelocityControl.kP; 
      slot0Configs.kI = Constants.VelocityControl.kI;
      slot0Configs.kD = Constants.VelocityControl.kD; 

      talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;     
      
      motor1.setNeutralMode(NeutralModeValue.Brake);
      //talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      motor1.getConfigurator().apply(talonFXConfigs);

      motor1.getPosition().setUpdateFrequency(150);
  }
  
   /**  Sets speed for right and left motors, left motor is reversed for intake to run in opposite direction
    * @param - speed
    */
   public void setMotor(double speed) {
      VelocityVoltage request = new VelocityVoltage(speed);
      motor1.setControl(request);
   }

   public void moveTo(double revolutions) {
      MotionMagicVoltage request = (new MotionMagicVoltage(revolutions)).withFeedForward(0.0);
      motor1.setControl(request);
   }

   public void setVelocity(double velocity) {
      //MotionMagicVelocityDutyCycle request = new MotionMagicVelocityDutyCycle(velocity);
      MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(velocity);

      motor1.setControl(request);

   }
   public void moveMotor(double speed) {
      VelocityVoltage request = new VelocityVoltage(speed);
      motor1.setControl(request);
   }

   public void stopMotor() {
      motor1.stopMotor();
   }

   
  





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
