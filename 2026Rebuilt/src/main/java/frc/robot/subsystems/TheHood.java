// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TheHood extends SubsystemBase {
  private final TalonFX hoodMotor;
  private final CANcoder absEncoder;

  public double revsToMove;

  public double currentPos;

  private InterpolatingDoubleTreeMap angleTable;
  private MotionMagicVoltage request;
  private MotionMagicVoltage maintainRequest;

  public TheHood() {
    hoodMotor = new TalonFX(Constants.Hood.HOOD_MOTOR_ID);
    absEncoder = new CANcoder(Constants.Hood.HOOD_CANCODER_ID);
    angleTable = new InterpolatingDoubleTreeMap();

    CANcoderConfiguration ccdConfigs = new CANcoderConfiguration();
    request = (new MotionMagicVoltage(revsToMove));
    maintainRequest = (new MotionMagicVoltage(currentPos));
    ccdConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    ccdConfigs.MagnetSensor.MagnetOffset = 
    -0.402099609375;
    ccdConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9;

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

    //current limits
    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 120;
    limitConfigs.StatorCurrentLimitEnable = true;

    hoodMotor.getConfigurator().apply(talonFXConfigs);
    hoodMotor.getConfigurator().apply(limitConfigs);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    absEncoder.getConfigurator().apply(ccdConfigs);

    setRelToAbs();
    populateTreeMap();
  }

  /**
   * @param revolutions - converted to revs
   * creates and sets a MotionMagicVoltage request with revs
   */
  public void moveTo(double revolutions) {
    revsToMove = revolutions*(Constants.Hood.GEAR_RATIO); 
    hoodMotor.setControl(request.withPosition(revsToMove));
  }

  /**
   * Sets current position of motor to current position of encoder
   * Use as reset encorder if needed in periodic(add if statement in periodic)
   */
  public void setRelToAbs(){
    hoodMotor.setPosition(getAbsPos()*Constants.Hood.GEAR_RATIO);
  }

  /**
   * Stops motor obviously :)
   */
  public void stop() {
    hoodMotor.stopMotor();
  }

  /**
   * @return position of absolute encoder
   */
  public double getAbsPos() {
    return (absEncoder.getAbsolutePosition().getValueAsDouble());
  }

  /**
   * @return position of absolute encoder in degrees
   */
  public double getAbsDeg() {
    return getAbsPos() * 360.0;
  }

  /**
   * @return true if the motor's position is within error threshold of the end goal
   */
  public boolean isReached() {
    return getAbsDeg() >= Constants.Hood.HOOD_MAX * 360;///Math.abs(((hoodMotor.getPosition().getValueAsDouble()/Constants.Hood.GEAR_RATIO)*360) - (revsToMove*360)) < 10.0;
  }

  /**
   * Populates angle tables with given distance and hood angles for vision tree map
   * Parameters for put method are (distance from hub(m), intended hood angle(degrees))
   */
  private void populateTreeMap() {
    angleTable.put(1.0, 20.0); 
  }

  /**
   * @param distanceToHub
   * @return angle of the hood based on distance in tree map
   */
  public double getNecessaryAngle(double distanceToHub) {
    return angleTable.get(distanceToHub);
  }

  /**
   * Creates and sets a MotionMagicVoltage request with the current position of the motor
   */
  public void maintainPosition() {
    currentPos = hoodMotor.getPosition().getValueAsDouble();
    hoodMotor.setControl(maintainRequest.withPosition(currentPos));
  }

  @Override
  public void periodic() {
     double position = hoodMotor.getPosition().getValueAsDouble();
     DogLog.log("Hood/Position (deg)", ((position/Constants.Hood.GEAR_RATIO)*360));
     DogLog.log("Hood/AbsEnc (deg)", getAbsDeg()); 
  }
}
