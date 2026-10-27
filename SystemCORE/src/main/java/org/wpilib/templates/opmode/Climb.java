// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.templates.opmode;

import org.wpilib.command2.SubsystemBase;

import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import first.robot.Constants;
import first.robot.Robot;

import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.discrete.DigitalInput;

@Teleop
public class Climb extends PeriodicOpMode {
  private final Robot robot;
  private TalonFX motor; 
  private TalonFXConfiguration config;
  private DigitalInput bottomSensor;
  private DigitalInput topSensor;
  private double rotations;
  private double appliedVoltage;
  private MotionMagicVoltage request;
  private final CANBus canbus = RobotContainer.CANBus;

  /** The Robot instance is passed into the opmode via the constructor. */
  public Climb(Robot robot) {
    this.robot = robot;

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

  public void moveTo(double inches) {  
    rotations = inches*(Constants.Climb.GEAR_RATIO/Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE);
    appliedVoltage = Constants.Climb.FEED_FORWARD;
    motor.setControl(request.withPosition(rotations));
  }

   public boolean atHeight(){
    double currentPosition = motor.getPosition().getValueAsDouble();
    return Math.abs(rotations - currentPosition) < Constants.Climb.ERROR_THRESHOLD;
   }

   public void setSafe(){
    if(getBottomSensor()){
      motor.setPosition(0);
    }
  }

  public boolean getBottomSensor(){
    return !bottomSensor.get();
  }

  public boolean getTopSensor(){
    return !topSensor.get();
  }

  public boolean isReachedTopSwitch(){
    return (getTopSensor());
  }

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
  public void disabledPeriodic() {
    /* Called on every robot period while the robot is disabled. */
  }

  @Override
  public void start() {
    /* Called once when the robot is enabled. */
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void end() {
    /* Called when the robot is disabled (after previously being enabled). */
  }

  @Override
  public void close() {
    /* Called when the opmode is de-selected / no additional methods will be called. */
  }
}
