// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.wpilib.command2.SubsystemBase;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import dev.doglog.DogLog;

import com.ctre.phoenix6.CANBus;


import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import first.robot.Constants;
import first.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import com.ctre.phoenix6.hardware.CANcoder;


public class Pivot extends SubsystemBase {
  private final CANBus canbus = RobotContainer.CANBus;
  private TalonFX motor;
  private MotionMagicVoltage request;
  private CANcoder absEncoder;
  private double revsToMove;

  private DynamicMotionMagicVoltage slowRequest = new DynamicMotionMagicVoltage(0, Constants.Pivot.CRUISE_VELOCITY_SLOW, Constants.Pivot.ACCELERATION_SLOW);
  private DynamicMotionMagicVoltage fastRequest = new DynamicMotionMagicVoltage(0, Constants.Pivot.CRUISE_VELOCITY_FAST, Constants.Pivot.ACCELERATION_FAST);


  /** Creates a new Pivot. */
  public Pivot() {
    motor = new TalonFX (Constants.Pivot.MOTOR_ID, canbus);
    request = new MotionMagicVoltage(revsToMove).withFeedForward(0.0);

    absEncoder = new CANcoder(Constants.Pivot.ENCODER_ID);
    CANcoderConfiguration ccdConfigs = new CANcoderConfiguration();
    ccdConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    //ccdConfigs.MagnetSensor.MagnetOffset = 0;
    ccdConfigs.MagnetSensor.MagnetOffset = Constants.Pivot.MAGNET_OFFSET;    

    var talonFXconfigs = new TalonFXConfiguration();

    var motorConfigs = new MotorOutputConfigs();
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;

    var slot0Configs = talonFXconfigs.Slot0;
    slot0Configs.kS = Constants.Pivot.kS; 
    slot0Configs.kV = Constants.Pivot.kV;
    slot0Configs.kA = Constants.Pivot.kA;
    slot0Configs.kP = Constants.Pivot.kP;
    slot0Configs.kI = Constants.Pivot.kI;
    slot0Configs.kD = Constants.Pivot.kD; 

    var motionMagicConfigs = talonFXconfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Pivot.CRUISE_VELOCITY_FAST;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Pivot.ACCELERATION_FAST;
    motionMagicConfigs.MotionMagicJerk = Constants.Pivot.JERK_FAST;

    absEncoder.getConfigurator().apply(ccdConfigs);
    motor.getConfigurator().apply(talonFXconfigs);
    motor.setNeutralMode(NeutralModeValue.Brake);

    // Make sure the encoder configuration applied before using it to sync the motor
    // position below.
    absEncoder.getAbsolutePosition().waitForUpdate(0.02);


    // StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    //   .getStructTopic("MyPose", Pose3d.struct).publish();
    // StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
    //   .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();
    setRelToAbs();
  }

  /**
   * set existing position of motors to be 0
   */
  public void resetRelEncoder(){
    motor.setPosition(0);
  }

  public boolean isReached() {
    double currentPos = motor.getPosition().getValueAsDouble()/Constants.Pivot.GEAR_RATIO*360;
    return isReached(currentPos);
  }
  
  /**
   * Determine if the position is within error threshold of target position. This version is used
   * when the rotor position is already known to avoid fetching it again.
   * @param currentPos Current rotor position
   * @return whether the position has reached the 
   */
  private boolean isReached(double currentPos) {
    double targetPosition = (revsToMove/Constants.Pivot.GEAR_RATIO)*360;
    double diff = Math.abs(currentPos - targetPosition);
    return diff < 10; 
    // double targetPosition = revsToMove/Constants.Pivot.GEAR_RATIO;
    // double diff = Math.abs(currentPos - targetPosition);
    // return diff < 0.1;
  }


  /**
   * @param rotations - converted to revs </p>
   * creates and sets a MotionMagicVoltage request with revs
   */
  public void moveTo(double rotations, boolean isSlow){
    revsToMove = rotations*Constants.Pivot.GEAR_RATIO;
    if (isSlow) {
      motor.setControl(slowRequest.withPosition(revsToMove));
    } else {
      motor.setControl(fastRequest.withPosition(revsToMove));
    }
  }



  /**
   * creates and sets a MotionMagicVoltage request with all 0 values
   */
  public void stop(){
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0.0).withFeedForward(0.0);
    motor.setControl(request.withVelocity(0));
  }

  /**
   * sets the current position of the absolute encoder to 0
   */
  public void resetAbsEncoder(){
    absEncoder.setPosition(0);
  }

  public void simulationInit(){
    // motorSim = motor.getSimState();
    // motorSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  // public void setRelToSafe() {
  //   if (
  //       absEncoder.getAbsolutePosition().getValueAsDouble()*360 > Constants.Pivot.SAFE - Constants.Pivot.THRESHOLD && 
  //       absEncoder.getAbsolutePosition().getValueAsDouble()*360 < Constants.Pivot.SAFE + Constants.Pivot.THRESHOLD
  //     ) {
  //     motor.setPosition(absEncoder.getAbsolutePosition().getValueAsDouble()*Constants.Pivot.GEAR_RATIO);
  //   }
  // }

  /**
   * sets current position of motor to current position of encoder
   */
  public void setRelToAbs() {
    motor.setPosition(absEncoder.getAbsolutePosition().getValueAsDouble()*Constants.Pivot.GEAR_RATIO);
  }

  /**
   * creates and sets a MotionMagicVoltage request with current position of motor
   */
  public void maintainPosition() {
    double currentPos = motor.getPosition().getValueAsDouble();
    motor.setControl(request.withPosition(currentPos));
  }

  // public void configDashboard(ShuffleboardTab tab) {
  //   // tab.addNumber("abs encoder pos", () -> absEncoder.getAbsolutePosition().getValueAsDouble()*360);
  //   // tab.addNumber("current pivot pos degrees", () -> (motor.getRotorPosition().getValueAsDouble()/Constants.Pivot.GEAR_RATIO)*360);
  //   // tab.addNumber("current pivot pos revs", () -> (motor.getRotorPosition().getValueAsDouble()/Constants.Pivot.GEAR_RATIO));
  //   // tab.addNumber("abs encoder pos revs", () -> absEncoder.getAbsolutePosition().getValueAsDouble());
  //   // tab.addNumber("target pivot pos degrees", () -> (revsToMove/Constants.Pivot.GEAR_RATIO)*360);
  //   // tab.addBoolean("pivot reached?", () -> isReached());
  // }

  public void periodic() {
    // This method will be called once per scheduler run
    //tab.addNumber("current pivot pos degrees", () -> (motor.getPosition().getValueAsDouble()/Constants.Pivot.GEAR_RATIO)*360);
    // double encoderPosition = absEncoder.getAbsolutePosition().getValueAsDouble();
    // double rotorPosition = motor.getPosition().getValueAsDouble();
    // DogLog.log("Pivot/abs encoder pos", encoderPosition*360);
    // DogLog.log("Pivot/current pos degrees", (rotorPosition/Constants.Pivot.GEAR_RATIO)*360);
    // DogLog.log("Pivot/current pos revs", (rotorPosition/Constants.Pivot.GEAR_RATIO));
    // DogLog.log("Pivot/abs encoder pos revs", encoderPosition);
    // DogLog.log("Pivot/pivot reached?", isReached(rotorPosition));
    // DogLog.log("Pivot/target pivot pos degrees", (revsToMove/Constants.Pivot.GEAR_RATIO)*360);

    if (Constants.Pivot.verboseLogging) {
      //DogLog.log("Pivot/stator current", motor.getStatorCurrent().getValueAsDouble());
    }
  }
}
