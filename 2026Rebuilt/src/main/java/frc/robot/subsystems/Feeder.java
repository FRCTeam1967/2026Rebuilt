// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Feeder extends SubsystemBase {
  private TalonFX motor;
  private final CANBus canbus = RobotContainer.CANBus;
  private final DoubleSubscriber feederSpeed = DogLog.tunable("Feeder/feederSpeed", Constants.Feeder.FEEDER_SPEED);

  /** Creates a new Feeder. */
  public Feeder() {
    motor = new TalonFX(Constants.Feeder.FEEDER_MOTOR_ID, canbus);

    var talonFXConfigurator = motor.getConfigurator();
    var motorConfigs = new MotorOutputConfigs();

    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfigurator.apply(motorConfigs);

    //DogLog.log("feeder speed", motor.getVelocity().getValueAsDouble());
  }

  /**
   * @param speed - sets motor to speed
   */
  public void setMotor(double speed){
    if (Constants.Feeder.verboseLogging) {
      //DogLog.log("Feeder/speed", speed);
      DogLog.log("Feeder/speed", feederSpeed.get());
    }
    //motor.set(speed);
    motor.set(feederSpeed.get());
  }

  /**
   * stops motor
   */
  public void stopMotor(){
    motor.stopMotor();
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.Feeder.verboseLogging) {
      DogLog.log("Feeder/stator current", motor.getStatorCurrent().getValueAsDouble());
    }
  }
}
