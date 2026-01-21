// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.AngularVelocity;


public class Hood extends SubsystemBase {
  
  private TalonFX hoodMotor;
  private DigitalInput limitSwitch;
  
  //private CANcode absEncode;
  private double revsToMove;
  private TalonFXConfiguration config;

  /** Creates a new Hood. */
  public Hood() {
    hoodMotor = new TalonFX(Constants.Hood.HOOD_MOTOR_ID);

    config = new TalonFXConfiguration();

    var talonFXConfigs = new TalonFXConfiguration();

    // Slot0 configs
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = Constants.Hood.kS;
    slot0Configs.kV = Constants.Hood.kV;
    slot0Configs.kA = Constants.Hood.kA;
    slot0Configs.kP = Constants.Hood.kP;
    slot0Configs.kI = Constants.Hood.kI;
    slot0Configs.kD = Constants.Hood.kD;

    // Motion Magic configs
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Hood.CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Hood.ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = Constants.Hood.JERK;

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //applies motion magic configs
    hoodMotor.getConfigurator().apply(talonFXConfigs);

    // Current limit
    talonFXConfigs.CurrentLimits = new CurrentLimitsConfigs()
      .withSupplyCurrentLimit(Constants.Hood.CURRENT_LIMIT)
      .withSupplyCurrentLimitEnable(true);

    // Apply configs
    hoodMotor.getConfigurator().apply(talonFXConfigs);

    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    // Zero if switch is already pressed at boot
    checkLimit();
  }
    //CHANGE AND REMOVE THESE METHODS FOR HOOD
  public void stop() {
      hoodMotor.stopMotor();
  }
  public void moveTo(double revolutions) {
    revsToMove = revolutions*(Constants.Hood.GEAR_RATIO);
    MotionMagicVoltage request = (new MotionMagicVoltage(revsToMove)).withFeedForward(0.0);
    hoodMotor.setControl(request);
  }
  
    /** checks if pivot has surpassed hard stop
    * if limit has been reached, sets pivot motor speed to 0 */
  public void checkLimit(){
    if (!limitSwitch.get()){
      hoodMotor.setPosition(0);
    }
  }

  public void resetEncoders() {
    hoodMotor.setPosition(0);
  }
    /** checks if the position the motor is at is within error threshold of the end goal */
  public boolean isReached() {
    return Math.abs(((hoodMotor.getRotorPosition().getValueAsDouble()/Constants.Hood.GEAR_RATIO)*360) - (revsToMove*360)) < 5.0;
  }

  public void maintainPosition() {
    moveTo(Constants.Hood.HOOD);
  }

  //THEORETICAL PART FOR FIND ANGLE!!! 
  
  double hoodAngle = 0.0;
  //find velocity later by testing :](velocity will be constant)
  //replace distance with tz later
  public double findAngle(double distance, double velocity)
  {
    hoodAngle = (Math.asin((9.8*distance)/(Math.pow(velocity, 2))/2))/(2*Math.PI);
    return hoodAngle;
  }
 //END OF THEORETICAL
  @Override
  public void periodic() {
    checkLimit();
  }
  
   /**Adds values to shuffleboard 
    *  we don't know if we need shuffle board values for right now on hood but this will be here just in case
    */
  public void configDashboard(ShuffleboardTab tab) {
    
  }

}

