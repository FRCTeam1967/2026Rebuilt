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

  private final CANBus canbus = new CANBus("Canivore");

  private double targetMotorRot = 0.0;

  public Hood() {
    hoodMotor = new TalonFX(Constants.Hood.HOOD_MOTOR_ID, canbus);
    absEncoder = new CANcoder(Constants.Hood.HOOD_CANCODER_ID, canbus);

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

    talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    talonFXConfigs.Feedback.SensorToMechanismRatio = 1.0;
    talonFXConfigs.Feedback.RotorToSensorRatio = 1.0;

    hoodMotor.getConfigurator().apply(talonFXConfigs);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    var ccdConfigs = new CANcoderConfiguration();
    ccdConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    //ccdConfigs.MagnetSensor.MagnetOffset = 0.337646484375;
    absEncoder.getConfigurator().apply(ccdConfigs);

    moveToDeg(Constants.Hood.HOOD_HOLD_DEG);
  }

  private static double clampDeg(double deg) { //putting mech limits for max and min degrees 
    return Math.max(Constants.Hood.MIN_DEG, Math.min(Constants.Hood.MAX_DEG, deg));
  }

  public static double motorRotToDeg(double motorRot) { //converts motor rotations to degrees
    return (motorRot / Constants.Hood.GEAR_RATIO) * 360.0;
  }

  public static double degToMotorRot(double deg) { //converts degrees to motor rotations
    return (deg / 360.0) * Constants.Hood.GEAR_RATIO;
  }

  public void stop() { //stop motor (obviously :) )
    hoodMotor.stopMotor();
  }

  public void moveToDeg(double degrees) { //goes to target degrees
    double clamped = clampDeg(degrees);
    targetMotorRot = degToMotorRot(clamped);
    hoodMotor.setControl(new MotionMagicVoltage(targetMotorRot).withFeedForward(0.0));
  }

  // public void maintainPosition() { //safety angle holding position
  //   moveToDeg(Constants.Hood.HOOD_HOLD_DEG);
  // }

  public double getHoodDeg() { //gets hood's angle in degrees 
    double motorRot = hoodMotor.getRotorPosition().getValueAsDouble();
    return motorRotToDeg(motorRot);
  }

  public double getAbsDeg() { //gets the absolute position from abs encoder
    return absEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  public boolean isReachedDeg(double toleranceDeg) { //checks if the hood has met the error threshold
    double targetDeg = motorRotToDeg(targetMotorRot);
    return Math.abs(getHoodDeg() - targetDeg) <= toleranceDeg;
  }

  public boolean isReached() { //uses tolerance deg to check if hood met error threshold :)
    return isReachedDeg(Constants.Hood.HOOD_TOLERANCE_DEG);
  }

  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Hood Position (deg)", () -> getHoodDeg());
    tab.addDouble("Hood Absolute (deg)", () -> getAbsDeg());
    tab.addDouble("Hood Target (deg)", () -> motorRotToDeg(targetMotorRot));
    tab.addBoolean("Hood at Target?", () -> isReached());
    tab.addDouble("Hood Rotor Rotations", () -> hoodMotor.getRotorPosition().getValueAsDouble());
  }

  @Override
  public void periodic() {}
}
