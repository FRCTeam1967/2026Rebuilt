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
import dev.doglog.DogLog;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {

  private final TalonFX hoodMotor;
  private final CANcoder absEncoder;

  public double revsToMove;

  private final CANBus canbus = new CANBus("CANivore");

  private InterpolatingDoubleTreeMap angleTable;

  //private double targetMotorRot = 0.0;

  public Hood() {
    hoodMotor = new TalonFX(Constants.Hood.HOOD_MOTOR_ID, canbus);

    //TODO: double check what the actual id is
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

    talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    talonFXConfigs.Feedback.SensorToMechanismRatio = 1.0;
    talonFXConfigs.Feedback.RotorToSensorRatio = 1.0;

    hoodMotor.getConfigurator().apply(talonFXConfigs);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    ccdConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; //change for hood testing
    
    //TODO: go to tuner x, rezero the absolute encoder, 
    //then get the magnet offset (which they provide) and add it here. 
    //now, whenever the hood is all the way down, the abs encoder 
    //will always read 0
    ccdConfigs.MagnetSensor.MagnetOffset =-0.313720703125;

    absEncoder.getConfigurator().apply(ccdConfigs);

    // moveToDeg(Constants.Hood.HOOD_HOLD_DEG);
    setRelToAbs();
    //resetEncoder();
    //stop();

    //TODO: populate this tree map for angles vs speeds 
    angleTable = new InterpolatingDoubleTreeMap();
    populateTreeMap();
  }

  public void moveTo(double revolutions) {
    revsToMove = revolutions*(Constants.Hood.GEAR_RATIO); 
    MotionMagicVoltage request = (new MotionMagicVoltage(revsToMove));
    //.withFeedForward(0.12); //changed this from 0.0 to 0.12 (value of kV)
    hoodMotor.setControl(request);
  }

  public void setRelToAbs(){
    //this should take care of the initial "set zero" method 
    //because the abs encoder will be zero at the start too
    hoodMotor.setPosition(absEncoder.getAbsolutePosition().getValueAsDouble()*Constants.Hood.GEAR_RATIO);
  }

  public void stop() { //stop motor (obviously :) )
    hoodMotor.stopMotor();
  }

  //changed this method from set zero to reset encoders, 
  //then put it in periodic to always check if we're at zero and to reset if so
  public void resetEncoder() {
    if (getAbsPos() == 0.0) {
      hoodMotor.setPosition(0.0);
    }
  }

  public double getAbsPos() { //gets the absolute position from abs encoder
    return (absEncoder.getAbsolutePosition().getValueAsDouble()*Constants.Hood.GEAR_RATIO);
  }

  public double getAbsDeg() { //gets the absolute position in degrees from abs encoder
    return absEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  /** checks if the position the motor is at is within error threshold of the end goal */
  public boolean isReached() {
      return Math.abs(((hoodMotor.getRotorPosition().getValueAsDouble()/Constants.Hood.GEAR_RATIO)*360) - (revsToMove*360)) < 10.0;
  }

  //TODO: fill in these values
  private void populateTreeMap() {
    //distance from hub (m), hood angle
    angleTable.put(1.0, 20.0); //example
  }

  //TODO: call this in robot container when setting speed
  public double getNecessaryAngle(double distanceToHub) {
    return angleTable.get(distanceToHub);
  }

  public void logAbsEncoder(){
    DogLog.log("AbsoluteHoodPosition", getAbsPos());
  }

  public void maintainPosition() {
    double currentPos = hoodMotor.getRotorPosition().getValueAsDouble();
    hoodMotor.setControl(new MotionMagicVoltage(currentPos));
  }

  // public void moveToDeg(double rotations) { //goes to target degrees
  //   // double clamped = clampDeg(degrees);
  //   hoodMotor.setControl(new MotionMagicVoltage(rotations).withFeedForward(10));
  // }

  // public double getHoodDeg() { //gets hood's angle in degrees 
  //   double motorRot = hoodMotor.getRotorPosition().getValueAsDouble();
  //   return motorRotToDeg(motorRot);
  // }

  // private static double clampDeg(double deg) { //putting mech limits for max and min degrees 
  //   return Math.max(Constants.Hood.MIN_DEG, Math.min(Constants.Hood.MAX_DEG, deg));
  // }

  // public static double motorRotToDeg(double motorRot) { //converts motor rotations to degrees
  //   return (motorRot / Constants.Hood.GEAR_RATIO) * 360.0;
  // }

  // public boolean isReachedDeg(double toleranceDeg) { //checks if the hood has met the error threshold
  //   double targetDeg = motorRotToDeg(targetMotorRot);
  //   return Math.abs(getHoodDeg() - targetDeg) <= toleranceDeg;
  // }

  // public boolean isReached() { //uses tolerance deg to check if hood met error threshold :)
  //   return isReachedDeg(Constants.Hood.HOOD_TOLERANCE_DEG);
  // }

  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Hood Position (deg)", () -> ((hoodMotor.getRotorPosition().getValueAsDouble()/Constants.Hood.GEAR_RATIO)*360));
    tab.addDouble("Hood Absolute (rot)", () -> getAbsPos());
    //tab.addDouble("Hood Target (deg)", () -> motorRotToDeg(targetMotorRot));
    tab.addBoolean("Hood at Target?", () -> isReached());
    tab.addDouble("Hood Rotor Rotations", () -> hoodMotor.getRotorPosition().getValueAsDouble());
  }

  // public void maintainPosition() {
  //     moveTo(Constants.Hood.HOOD_HOLD_DEG);
  //  }

  @Override
  public void periodic() {
    //resetEncoder();
  }
}
