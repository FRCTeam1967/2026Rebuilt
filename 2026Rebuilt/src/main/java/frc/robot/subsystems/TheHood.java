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
import frc.robot.RobotContainer;

// WELCOME TO DA HOOD

public class TheHood extends SubsystemBase {
  private final TalonFX hoodMotor;
  private final CANcoder absEncoder;

  public double revsToMove;
  public double currentPos;

  private final CANBus canbus = RobotContainer.CANBus;

  private InterpolatingDoubleTreeMap angleTable;
  private MotionMagicVoltage request;
  private MotionMagicVoltage maintainRequest;

  public TheHood() {
    hoodMotor = new TalonFX(Constants.Hood.HOOD_MOTOR_ID, canbus);
    absEncoder = new CANcoder(Constants.Hood.HOOD_CANCODER_ID, canbus);
    angleTable = new InterpolatingDoubleTreeMap();

    CANcoderConfiguration ccdConfigs = new CANcoderConfiguration();
    request = (new MotionMagicVoltage(revsToMove));
    maintainRequest = (new MotionMagicVoltage(currentPos));

    ccdConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; //change for hood testing
    ccdConfigs.MagnetSensor.MagnetOffset =-0.3056640625;
    ccdConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

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
    //.withFeedForward(0.12); //changed this from 0.0 to 0.12 (value of kV)
    hoodMotor.setControl(request);
  }

  /**
   * sets current position of motor to current position of encoder </p>
   * USE AS RESET ENCODER IF NEEDED IN PERIODIC (add an if in periodic, instead of creating a new method)
   */
  public void setRelToAbs(){
    //this should take care of the initial "set zero" method 
    //because the abs encoder will be zero at the start too
    hoodMotor.setPosition(getAbsPos()*Constants.Hood.GEAR_RATIO);
  }

  /**
   * stop motor obviously :)
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
    return getAbsDeg() >= Constants.Hood.HOOD_MAX * 360;///Math.abs(((hoodMotor.getRotorPosition().getValueAsDouble()/Constants.Hood.GEAR_RATIO)*360) - (revsToMove*360)) < 10.0;
  }

  /**
   * populates angle tables with given distance and hood angles
   */
  private void populateTreeMap() {
    //distance from hub (m), hood angle
    angleTable.put(1.0, 20.0); //example
  }

  //TODO: call this in robot container when setting speed
  /**
   * @param distanceToHub
   * @return angle of the hood based on distance in tree map
   */
  public double getNecessaryAngle(double distanceToHub) {
    return angleTable.get(distanceToHub);
  }

  /**
   * log value of absolute encoder to doglog
   */
  public void logRequest(){
    DogLog.log("HoodRequest", Constants.Hood.HOOD_ANGLE);
  }

  /**
   * creates and sets a MotionMagicVoltage request with current position of motor
   */
  public void maintainPosition() {
    currentPos = hoodMotor.getRotorPosition().getValueAsDouble();
    hoodMotor.setControl(maintainRequest);
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
    tab.addDouble("Hood AbsEnc (deg)", () -> getAbsDeg()); //TODO: in changed code, this was indicated to be rotations, not degrees
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
