// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

import com.ctre.phoenix6.sim.TalonFXSimState;
import static edu.wpi.first.units.Units.Volts;

public class FlywheelShooter extends SubsystemBase {

  private TalonFX flywheelMotor1;
  private TalonFX flywheelMotor2;
  private TalonFX flywheelMotor3;

  private static final double kSimDt = 0.02;

  private double simRotorPosRot = 0.0;

  private final DCMotorSim flywheelSim = new DCMotorSim(
   LinearSystemId.createDCMotorSystem(
      DCMotor.getKrakenX60Foc(1), 0.001, Constants.FlywheelShooter.GEAR_RATIO
   ),
   DCMotor.getKrakenX60Foc(1)
);

  /** Creates a new FlywheelShooter. */
  public FlywheelShooter() {
    flywheelMotor1 = new TalonFX(Constants.FlywheelShooter.FLYWHEELSHOOTER_MOTOR1_ID);
    flywheelMotor2 = new TalonFX(Constants.FlywheelShooter.FLYWHEELSHOOTER_MOTOR2_ID);
    flywheelMotor3 = new TalonFX(Constants.FlywheelShooter.FLYWHEELSHOOTER_MOTOR3_ID);

      var talonFXConfigs = new TalonFXConfiguration();

      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = Constants.FlywheelShooter.kS; 
      slot0Configs.kV = Constants.FlywheelShooter.kV; 
      slot0Configs.kA = Constants.FlywheelShooter.kA; 
      slot0Configs.kP = Constants.FlywheelShooter.kP; 
      slot0Configs.kI = Constants.FlywheelShooter.kI;
      slot0Configs.kD = Constants.FlywheelShooter.kD; 
  
      // set Motion Magic settings
      var motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.FlywheelShooter.CRUISE_VELOCITY;
      motionMagicConfigs.MotionMagicAcceleration = Constants.FlywheelShooter.ACCELERATION;
      motionMagicConfigs.MotionMagicJerk = Constants.FlywheelShooter.JERK;

      talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

      flywheelMotor1.setNeutralMode(NeutralModeValue.Brake);
      flywheelMotor2.setNeutralMode(NeutralModeValue.Brake);
      flywheelMotor3.setNeutralMode(NeutralModeValue.Brake);
      //talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      flywheelMotor1.getConfigurator().apply(talonFXConfigs);
      flywheelMotor2.getConfigurator().apply(talonFXConfigs);
      flywheelMotor3.getConfigurator().apply(talonFXConfigs);
  }

  public void setMotor(double speed) {
      flywheelMotor1.set(speed);
      flywheelMotor2.set(speed);
      flywheelMotor3.set(speed);
  }

  //public void moveTo(double revolutions) {
      //MotionMagicVoltage request = (new MotionMagicVoltage(revolutions)).withFeedForward(0.0);
      //flywheelMotor.setControl(request);
  //}

  public void setVelocity(double velocity) {
      //MotionMagicVelocityDutyCycle request = new MotionMagicVelocityDutyCycle(velocity);
      MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(velocity);

      flywheelMotor1.setControl(request);
      flywheelMotor2.setControl(request);
      flywheelMotor3.setControl(request);
  }

  //public void moveMotor(double speed) {
      //VelocityVoltage request = new VelocityVoltage(speed);
      //intakeMotor.setControl(request);
   //}
  
   /** Stops both the left motor and right motor */
   public void stopMotor() {
      flywheelMotor1.stopMotor();
      flywheelMotor2.stopMotor();
      flywheelMotor3.stopMotor();
   }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    @Override
  public void simulationPeriodic() {
    var sim1 = flywheelMotor1.getSimState();
    var sim2 = flywheelMotor2.getSimState();
    var sim3 = flywheelMotor3.getSimState();

    double batteryV = RobotController.getBatteryVoltage();
    sim1.setSupplyVoltage(batteryV);
    sim2.setSupplyVoltage(batteryV);
    sim3.setSupplyVoltage(batteryV);

    double motorVolts =
        sim1.getMotorVoltageMeasure().in(Volts);

    flywheelSim.setInputVoltage(motorVolts);
    flywheelSim.update(kSimDt);

    double mechRps =
        flywheelSim.getAngularVelocityRadPerSec()
        / (2.0 * Math.PI);

    double rotorRps =
        mechRps * Constants.FlywheelShooter.GEAR_RATIO;

    simRotorPosRot += rotorRps * kSimDt;

    sim1.setRotorVelocity(rotorRps);
    sim2.setRotorVelocity(rotorRps);
    sim3.setRotorVelocity(rotorRps);

    sim1.setRawRotorPosition(simRotorPosRot);
    sim2.setRawRotorPosition(simRotorPosRot);
    sim3.setRawRotorPosition(simRotorPosRot);

    SmartDashboard.putNumber("Flywheel/RotorRPS", rotorRps);
  }
}
