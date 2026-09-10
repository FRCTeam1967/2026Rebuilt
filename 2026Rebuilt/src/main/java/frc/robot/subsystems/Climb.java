// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class Climb extends SubsystemBase {  
  private TalonFX motor; 
  private TalonFXConfiguration config;
  private DigitalInput bottomSensor;
  private DigitalInput topSensor;
  private double rotations;
  private double appliedVoltage;
  private MotionMagicVoltage request;

  public Climb() {
    motor = new TalonFX(Constants.Climb.MOTOR_ID);
    config = new TalonFXConfiguration();
    bottomSensor = new DigitalInput(Constants.Climb.BOTTOM_SENSOR_CHANNEL);
    topSensor = new DigitalInput(Constants.Climb.TOP_SENSOR_CHANNEL);
    request = new MotionMagicVoltage(rotations).withFeedForward(Constants.Climb.FEED_FORWARD);

    CANcoderConfiguration ccdConfigs = new CANcoderConfiguration();


    ccdConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    // var limitConfigs = new CurrentLimitsConfigs();
    // limitConfigs.StatorCurrentLimit = 1;
    // limitConfigs.StatorCurrentLimitEnable = true;
    
    config.Slot0.kP = Constants.Climb.kP;
    config.Slot0.kI = Constants.Climb.kI;
    config.Slot0.kD = Constants.Climb.kD;
    config.Slot0.kS = Constants.Climb.kS;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Climb.CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = Constants.Climb.ACCELERATION;
    
    config.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Climb.CURRENT_LIMIT));config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);
    
  }
   

  /**
   * resets the climb motor's encoder position to be 0
   */
  public void resetEncoders() {
    motor.setPosition(0);
  }

  /**
   * @param inches - converted to rotations </p>
   * sets appliedVoltage to feedforward </p>
   * creates and sets a MotionMagicVoltage request with rotations and feedforward
   */
  public void moveTo(double inches) {  
    rotations = inches*(Constants.Climb.GEAR_RATIO/Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE);
    appliedVoltage = Constants.Climb.FEED_FORWARD;
    motor.setControl(request.withPosition(rotations));
  }

  /**
   * @return true if motor's current position is within error threshold of target height
   */
  public boolean atHeight(){
    double currentPosition = motor.getPosition().getValueAsDouble();
    return Math.abs(rotations - currentPosition) < Constants.Climb.ERROR_THRESHOLD;

  } 

  /**
   * resets the climb motor position to 0 when the bottom sensor is triggered
   */
  public void setSafe(){
    if(getBottomSensor()){
      motor.setPosition(0);
    }
  }
  /**
   * gets current state of the bottom limit sensor
   * @return true if the bottom sensor is triggered and false otherwise
   */
  public boolean getBottomSensor(){
    return !bottomSensor.get();
  }
   /**
   * gets current state of the top limit sensor
   * @return true if the top sensor is triggered and false otherwise
   */
    public boolean getTopSensor(){
    return !topSensor.get();
  }
  /**
   * 
   * @return true if the top switch has been reached and false otherwise
   */
  public boolean isReachedTopSwitch(){
    return (getTopSensor());
  }
  
  /**
   * 
   * @return true if the bottom switch has been reached and false otherwise
   */
  public boolean isReachedBottomSwitch(){
    return (getBottomSensor());
  }

  /**
   * Stops motor
   */
  public void stopMotor() {
    motor.stopMotor();
  }


  @Override
  public void periodic() {
    // double rotorPosition = motor.getPosition().getValueAsDouble();
    // DogLog.log("Climb/at height", Math.abs(rotations) - Math.abs(rotorPosition) < Constants.Climb.ERROR_THRESHOLD);
    // DogLog.log("Climb/target rotations", rotations);
    // DogLog.log("Climb/rotations", rotorPosition);
    // DogLog.log("Climb/inches", rotorPosition/(Constants.Climb.GEAR_RATIO/Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE));
    // DogLog.log("Climb/bottom sensor", getBottomSensor());
    // DogLog.log("Climb/top sensor", getTopSensor());

    if (Constants.Climb.verboseLogging) {
      // DogLog.log("Climb/stator current", motor.getStatorCurrent().getValueAsDouble());
    }

    //setSafe();
  }
}