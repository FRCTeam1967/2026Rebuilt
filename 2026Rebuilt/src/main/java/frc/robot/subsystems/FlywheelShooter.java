// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volts;

public class FlywheelShooter extends SubsystemBase {

  private TalonFX flywheelMotor1;
  private TalonFX flywheelMotor2;


  private static final double kSimDt = 0.02;
  private double simRotorPosRot = 0.0;

  private final DCMotorSim flywheelSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60Foc(1),
          0.001,
          Constants.FlywheelShooter.GEAR_RATIO
      ),
      DCMotor.getKrakenX60Foc(1)
  );


  private final Mechanism2d shooterMech = new Mechanism2d(2, 2);
  private final MechanismRoot2d shooterRoot = shooterMech.getRoot("shooterRoot", 1, 1);

  // A "spoke" that rotates to show the flywheel spinning
  private final MechanismLigament2d flywheelSpoke =
      shooterRoot.append(new MechanismLigament2d(
          "flywheelShooter",
          0.8,  // length
          0.0,  // initial angle (deg)
          6,    // line thickness
          new Color8Bit(Color.kOrange)
      ));

  private double spokeAngleDeg = 0.0;

  /** Creates a new FlywheelShooter. */
  public FlywheelShooter() {
    flywheelMotor1 = new TalonFX(Constants.FlywheelShooter.FLYWHEELSHOOTER_MOTOR1_ID);
    flywheelMotor2 = new TalonFX(Constants.FlywheelShooter.FLYWHEELSHOOTER_MOTOR2_ID);

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = Constants.FlywheelShooter.kS;
    slot0Configs.kV = Constants.FlywheelShooter.kV;
    slot0Configs.kA = Constants.FlywheelShooter.kA;
    slot0Configs.kP = Constants.FlywheelShooter.kP;
    slot0Configs.kI = Constants.FlywheelShooter.kI;
    slot0Configs.kD = Constants.FlywheelShooter.kD;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.FlywheelShooter.CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = Constants.FlywheelShooter.ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = Constants.FlywheelShooter.JERK;

    talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    flywheelMotor1.setNeutralMode(NeutralModeValue.Brake);
    flywheelMotor2.setNeutralMode(NeutralModeValue.Brake);

    flywheelMotor1.getConfigurator().apply(talonFXConfigs);
    flywheelMotor2.getConfigurator().apply(talonFXConfigs);

    SmartDashboard.putData("ShooterMech2d", shooterMech);
  }

  public void setMotor(double speed) {
    flywheelMotor1.set(speed);
    flywheelMotor2.set(speed);
  }

  public void setVelocity(double velocity) {
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(velocity);
    flywheelMotor1.setControl(request);
    flywheelMotor2.setControl(request);
  }

  public void stopMotor() {
    flywheelMotor1.stopMotor();
    flywheelMotor2.stopMotor();
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {

    // If disabled, hard-reset sim outputs so GUI + SmartDashboard + Mechanism2d go to 0
    if (RobotState.isDisabled()) {
      stopMotor();

      // Reset sim model
      flywheelSim.setInputVoltage(0.0);
      flywheelSim.update(kSimDt);

      simRotorPosRot = 0.0;

      var sim1 = flywheelMotor1.getSimState();
      var sim2 = flywheelMotor2.getSimState();

      double batteryV = RobotController.getBatteryVoltage();
      sim1.setSupplyVoltage(batteryV);
      sim2.setSupplyVoltage(batteryV);

      sim1.setRotorVelocity(0.0);
      sim2.setRotorVelocity(0.0);

      sim1.setRawRotorPosition(0.0);
      sim2.setRawRotorPosition(0.0);

      // Reset Mechanism2d
      spokeAngleDeg = 0.0;
      flywheelSpoke.setAngle(0.0);

      SmartDashboard.putNumber("Flywheel/RotorRPS", 0.0);
      return;
    }

    var sim1 = flywheelMotor1.getSimState();
    var sim2 = flywheelMotor2.getSimState();

    double batteryV = RobotController.getBatteryVoltage();
    sim1.setSupplyVoltage(batteryV);
    sim2.setSupplyVoltage(batteryV);

    double motorVolts = sim1.getMotorVoltageMeasure().in(Volts);

    flywheelSim.setInputVoltage(motorVolts);
    flywheelSim.update(kSimDt);

    double mechRps = flywheelSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double rotorRps = mechRps * Constants.FlywheelShooter.GEAR_RATIO;

    simRotorPosRot += rotorRps * kSimDt;

    sim1.setRotorVelocity(rotorRps);
    sim2.setRotorVelocity(rotorRps);

    sim1.setRawRotorPosition(simRotorPosRot);
    sim2.setRawRotorPosition(simRotorPosRot);

    // Mechanism2d: rotate the spoke based on rotor speed
    spokeAngleDeg = (spokeAngleDeg + rotorRps * 360.0 * kSimDt) % 360.0;
    flywheelSpoke.setAngle(spokeAngleDeg);

    SmartDashboard.putNumber("Flywheel/RotorRPS", rotorRps);
    SmartDashboard.putNumber("Flywheel/MotorVolts", motorVolts);
    SmartDashboard.putNumber("Flywheel/SpokeAngleDeg", spokeAngleDeg);
  }
}
