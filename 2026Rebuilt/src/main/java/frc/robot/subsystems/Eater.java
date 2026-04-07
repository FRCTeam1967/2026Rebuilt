// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BulkCANSignalUpdater.CANBusType;

public class Eater extends SubsystemBase {
  private TalonFX motor;
  private final CANBus canbus = RobotContainer.CANBus;
  private final DoubleSubscriber intakeSpeed = DogLog.tunable("Eater/intakeSpeed", Constants.Eater.EATER_MOTOR_SPEED);

  /**
   * Signals and their local values. It is the responsibility of init() to create and register the
   * signals we care about. And it is the responsibility of periodic() to call updateInputs() which
   * is responsible for updating the values associated with each signal. No other part of the subsystem
   * should fetch a signal from the motor, nor look at the value of the signal directly. The rest of the
   * subsystem should exclusively use the value that updateInputs() updates.
   */
  private StatusSignal<Current> supplyCurrentSignal;
  private double supplyCurrent;

  /** Creates a new Intake. */
  public Eater() {
    motor = new TalonFX(Constants.Eater.EATER_MOTOR_ID);
    var talonFXConfigurator = motor.getConfigurator();

    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 60;
    limitConfigs.StatorCurrentLimitEnable = true;

    var motorConfigs = new MotorOutputConfigs();

    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfigurator.apply(motorConfigs);
    talonFXConfigurator.apply(limitConfigs);

    createSignals();
  }

  /**
   * @param speed - sets motor to speed
   */
  public void setMotor(double speed) {
    motor.set(speed);
    //motor.set(intakeSpeed.get());
    // DogLog.log("Eater/intake desired speed", intakeSpeed.get());
  }

  public boolean isStalling() {
    return (supplyCurrent > 75.0); 
  }

  
  /**
   * stops motor
   */
  public void stopMotor(){
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    updateInputs();

    // This method will be called once per scheduler run
    if (Constants.Eater.verboseLogging) {
      // DogLog.log("Eater/stator current", motor.getStatorCurrent().getValueAsDouble());
    }
  }

  private void createSignals() {
    supplyCurrentSignal = motor.getSupplyCurrent();
    supplyCurrentSignal.setUpdateFrequency(Constants.CANUpdateFrequencies.nonCriticalSignal);

    BulkCANSignalUpdater signalUpdater = BulkCANSignalUpdater.getInstance();
    signalUpdater.registerSignals(CANBusType.RIO, supplyCurrentSignal);
    signalUpdater.optimizeDevices(motor);
  }

  private void updateInputs() {
    supplyCurrent = supplyCurrentSignal.getValueAsDouble();
  }
}