// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {

  private final TalonFX hoodMotor;
  private final CANcoder absEncoder;

  public double revsToMove;

  private final CANBus canbus = new CANBus("CANivore");

  //private double targetMotorRot = 0.0;

  public Hood() {
    hoodMotor = new TalonFX(Constants.Hood.HOOD_MOTOR_ID, canbus);
    absEncoder = new CANcoder(Constants.Hood.HOOD_CANCODER_ID, canbus);
    CANcoderConfiguration ccdConfigs = new CANcoderConfiguration();

    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.Slot0.kS = Constants.Hood.kS;
    talonFXConfigs.Slot0.kV = Constants.Hood.kV;
    talonFXConfigs.Slot0.kA = Constants.Hood.kA;
    talonFXConfigs.Slot0.kP = Constants.Hood.kP;
    talonFXConfigs.Slot0.kI = Constants.Hood.kI;
    talonFXConfigs.Slot0.kD = Constants.Hood.kD;

    talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Hood.CRUISE_VELOCITY;
    talonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.Hood.ACCELERATION;
    talonFXConfigs.MotionMagic.MotionMagicJerk = Constants.Hood.JERK;

    //ccdConfigs.MagnetSensor.MagnetOffset = Constants.Hood.OFFSET;

    talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    talonFXConfigs.Feedback.SensorToMechanismRatio = 1.0;
    talonFXConfigs.Feedback.RotorToSensorRatio = 1.0;

    hoodMotor.getConfigurator().apply(talonFXConfigs);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    ccdConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    //ccdConfigs.MagnetSensor.MagnetOffset = 0.337646484375;
    absEncoder.getConfigurator().apply(ccdConfigs);

    // moveToDeg(Constants.Hood.HOOD_HOLD_DEG);
    setZero();
    //stop();
  }

  public void moveTo(double revolutions) {
    revsToMove = revolutions*(Constants.Hood.GEAR_RATIO); 
    MotionMagicVoltage request = (new MotionMagicVoltage(revsToMove)).withFeedForward(0.0);
    hoodMotor.setControl(request);
   }

  // private static double clampDeg(double deg) { //putting mech limits for max and min degrees 
  //   return Math.max(Constants.Hood.MIN_DEG, Math.min(Constants.Hood.MAX_DEG, deg));
  // }

  // public static double motorRotToDeg(double motorRot) { //converts motor rotations to degrees
  //   return (motorRot / Constants.Hood.GEAR_RATIO) * 360.0;
  // }

  public void stop() { //stop motor (obviously :) )
    hoodMotor.stopMotor();
  }
  public void setZero() {
    hoodMotor.setPosition(0);
    absEncoder.setPosition(Constants.Hood.HOOD_MAX);
  }

  // public void moveToDeg(double rotations) { //goes to target degrees
  //   // double clamped = clampDeg(degrees);
  //   hoodMotor.setControl(new MotionMagicVoltage(rotations).withFeedForward(10));
  // }

  // public double getHoodDeg() { //gets hood's angle in degrees 
  //   double motorRot = hoodMotor.getRotorPosition().getValueAsDouble();
  //   return motorRotToDeg(motorRot);
  // }

  public double getAbsDeg() { //gets the absolute position from abs encoder
    return absEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  // public boolean isReachedDeg(double toleranceDeg) { //checks if the hood has met the error threshold
  //   double targetDeg = motorRotToDeg(targetMotorRot);
  //   return Math.abs(getHoodDeg() - targetDeg) <= toleranceDeg;
  // }

  // public boolean isReached() { //uses tolerance deg to check if hood met error threshold :)
  //   return isReachedDeg(Constants.Hood.HOOD_TOLERANCE_DEG);
  // }

  /** checks if the position the motor is at is within error threshold of the end goal */
   public boolean isReached() {
      return Math.abs(((hoodMotor.getRotorPosition().getValueAsDouble()/Constants.Hood.GEAR_RATIO)*360) - (revsToMove*360)) < 10.0;
   }

  public void setRelToAbs(){
    hoodMotor.setPosition(absEncoder.getAbsolutePosition().getValueAsDouble()*Constants.Hood.GEAR_RATIO);
  }

  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Hood Position (deg)", () -> ((hoodMotor.getRotorPosition().getValueAsDouble()/Constants.Hood.GEAR_RATIO)*360));
    tab.addDouble("Hood Absolute (deg)", () -> getAbsDeg());
    //tab.addDouble("Hood Target (deg)", () -> motorRotToDeg(targetMotorRot));
    tab.addBoolean("Hood at Target?", () -> isReached());
    tab.addDouble("Hood Rotor Rotations", () -> hoodMotor.getRotorPosition().getValueAsDouble());
  }

  // public void maintainPosition() {
  //     moveTo(Constants.Hood.HOOD_HOLD_DEG);
  //  }

  @Override
  public void periodic() {}
}
