// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import dev.doglog.DogLog;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

public class IntakeShooter extends SubsystemBase {
  private TalonFX motor;
  private final RobotContainer m_robotContainer; 

  private final CANBus canbus = RobotContainer.CANBus;

  private final DoubleSubscriber yeeterAcceleration = DogLog.tunable("Shooter/yeeterAcceleration", Constants.IntakeShooter.SHOOTER_ACCELERATION);
  private final DoubleSubscriber cruiseVelocity = DogLog.tunable("Shooter/cruiseVelocity", Constants.IntakeShooter.CRUISE_VELOCITY);
  private final DoubleSubscriber mmAcceleration = DogLog.tunable("Shooter/mmAcceleration", Constants.IntakeShooter.ACCELERATION);

  private MotionMagicVelocityTorqueCurrentFOC torqueRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

  /** Creates a new FlywheelShooter. */
  public IntakeShooter(RobotContainer robotContainer){
    motor = new TalonFX(Constants.IntakeShooter.INTAKE_SHOOTER_MOTOR_ID);
    m_robotContainer = robotContainer; 

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = Constants.IntakeShooter.kS;
    slot0Configs.kV = Constants.IntakeShooter.kV;
    slot0Configs.kA = Constants.IntakeShooter.kA;
    slot0Configs.kP = Constants.IntakeShooter.kP;
    slot0Configs.kI = Constants.IntakeShooter.kI;
    slot0Configs.kD = Constants.IntakeShooter.kD;

    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 60;
    limitConfigs.StatorCurrentLimitEnable = true;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.IntakeShooter.CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = Constants.IntakeShooter.ACCELERATION;

    // motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity.get();
    // motionMagicConfigs.MotionMagicAcceleration = mmAcceleration.get();
    //motionMagicConfigs.MotionMagicJerk = Constants.FlywheelShooter.JERK;

    talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    motor.setNeutralMode(NeutralModeValue.Coast);

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(talonFXConfigs);

    motor.getConfigurator().apply(limitConfigs);
  }

  /**
   * set velocity of the motor with MotionMagicVelocityVoltage requests params </p>
   * creates torque request but doesn't use it?
   * velocitySupplier should probably stored as a member field and then used in reachedShooterSpeed() rather than 
   * having reachedShooterSpeed() reach into robot container to get the Visabelle. You'd probably want to have
   * stopMotor() then set velocitySupplier to NULL or set it to a DoubleSupplier that returns a constant value of 0.
   * @param velocity
   * @param acceleration
   */

  //for shooter
  public void setVelocity(DoubleSupplier velocitySupplier, double acceleration) {

    double velocity = velocitySupplier.getAsDouble();
    DogLog.log("Shooter/target speed (set)", velocity);

    motor.setControl(torqueRequest.withVelocity(velocity).withAcceleration(yeeterAcceleration.get()));
  }

  //for intake
  public void setMotor(double speed) {
    motor.set(speed);
    //motor.set(intakeSpeed.get());
    // DogLog.log("Eater/intake desired speed", intakeSpeed.get());
  }

  public boolean isStalling() {
    return (motor.getSupplyCurrent().getValueAsDouble() > 75.0); 
  }

  /**
   * @return true if current speed of yeeter is >= threshold speed
   */
  
    public boolean reachedShooterSpeed(double desiredSpeed) {
      double currentSpeed = getCurrentVelocity();
      double difference = Math.abs(desiredSpeed - currentSpeed);
      return (difference <= Constants.IntakeShooter.SHOOTER_THRESHOLD_ERROR); 
  }
 

  /**
   * @return average velocity of both motors
   */
  public double getCurrentVelocity() {
    double currentVelocity = motor.getVelocity().getValueAsDouble();
    return(currentVelocity);
  }

  /**
   * stops both motors
   */
  public void stopMotor() {
    motor.stopMotor();
  }


  /**
   * @param motor
   * @return velocity as double of motor
   */
  public double getMotorVelocity(TalonFX motor) {
    return (motor.getVelocity().getValueAsDouble());
  }

  @Override
  public void periodic() {
  }
}