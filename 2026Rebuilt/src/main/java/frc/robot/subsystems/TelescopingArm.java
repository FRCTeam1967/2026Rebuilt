// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TelescopingArm extends SubsystemBase {
  private TalonFX motor;
  private TalonFXConfiguration config;
  private boolean isRight;
  private DigitalInput sensor;
  private boolean enabled;
  
  /**
   * Creates new Climb
   * <p> Initializing and configuring motor for Motion Magic, initializing sensor
   * @param motorID - port for motor
   */
  public TelescopingArm(int motorID) {
    motor = new TalonFX(motorID);
    config = new TalonFXConfiguration();
    enabled = false;

    sensor = new DigitalInput(Constants.TelescopingArm.DIGITAL_INPUT_ID);
    //config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.TelescopingArm.CURRENT_LIMIT));
    motor.getConfigurator().apply(config);
  }
  
  /** Zeros encoder at starting position (latched) */
  public void setEncoderOffset() {
    motor.setPosition(0);
  }
  
  /**
   * @param speed - speed of motor
   */
  public void runMotor(DoubleSupplier speed) {
    double deadbandedSpeed = MathUtil.applyDeadband(speed.getAsDouble(), Constants.TelescopingArm.DEADBAND);
    
    if(deadbandedSpeed > 0) {
      motor.set(deadbandedSpeed * Constants.TelescopingArm.UNWIND_FACTOR);
    } else {
      motor.set(deadbandedSpeed * Constants.TelescopingArm.WIND_FACTOR);
    }
  }

  /** Changes permission to run climb motors */
  public void changeStatus() {
    enabled = !enabled;
  }

  /** @return mode - true means climbing is allowed */
  public boolean isEnabled(){
    return enabled;
  }
  
  /** Stops climb motor */
  public void stop() {
    motor.stopMotor();
  }

  /** @return motor rotor position */
  public double getEncoderCount(){
    return motor.getRotorPosition().getValueAsDouble();
  }
  
  /** @return sensor value (false when triggered) */
  public boolean getSensorValue() {
    return !sensor.get();
  }
  
  /**
   * Displays value of relative encoder on Shuffleboard
   * @param tab - ShuffleboardTab to add values to
   */
  public void configDashboard(ShuffleboardTab tab) {
    if(isRight){
      tab.addDouble("Right Climb Rel Encoder", () -> motor.getRotorPosition().getValueAsDouble());
      tab.addBoolean("Right Climb Sensor", () -> getSensorValue());
      tab.addBoolean("Enabled On", () -> isEnabled());
    } else {
      tab.addDouble("Left Climb Rel Encoder", () -> motor.getRotorPosition().getValueAsDouble());
      tab.addBoolean("Left Climb Sensor", () -> getSensorValue());
    }      
  }
  
  @Override
  public void periodic() {}
}