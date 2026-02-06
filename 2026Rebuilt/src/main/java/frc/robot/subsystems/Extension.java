// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;


public class Extension extends SubsystemBase {
  private TalonFX motor;
  public double revsToMove;
  private SingleJointedArmSim armSim;
  private Mechanism2d mech2d = new Mechanism2d(1, 1);
  private MechanismLigament2d arm = mech2d.getRoot("pivot", 0.5, 0.5)
      .append(new MechanismLigament2d("arm", 0.5, 0));
  private TalonFXSimState motorSim;
  private double simRotorPosition = 0.0;
  private double simAppliedVoltage = 0.0;
  /** Creates a new Pivot. */
  public Extension() {
    motor = new TalonFX (Constants.Extension.EXTENSION_MOTOR_ID);
    motorSim = motor.getSimState();
    var talonFXconfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXconfigs.Slot0;
    slot0Configs.kS = Constants.Extension.kS; 
    slot0Configs.kV = Constants.Extension.kV;
    slot0Configs.kA = Constants.Extension.kA;
    slot0Configs.kP = Constants.Extension.kP;
    slot0Configs.kI = Constants.Extension.kI;
    slot0Configs.kD = Constants.Extension.kD; 
    
    var motionMagicConfigs = talonFXconfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Extension.CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Extension.ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = Constants.Extension.JERK;

    talonFXconfigs.Feedback.SensorToMechanismRatio = Constants.Extension.GEAR_RATIO;

    motor.getConfigurator().apply(talonFXconfigs);

    motor.setNeutralMode(NeutralModeValue.Brake);

    armSim = new SingleJointedArmSim(
    DCMotor.getKrakenX60(1), // Motor
    Constants.Extension.GEAR_RATIO,  // Gear ratio
    5.0, // Moment of inertia (kg·m²)
    0.5,  // Arm length (m)
    0.0,  // Min angle (rad)
    Math.PI,  // Max angle (rad)
    true, // Simulate gravity
    0.0 // Initial angle (rad)
    );  
  }

  public void simulationInit(){
    motorSim = motor.getSimState();
    motorSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

    
  public void resetEncoders(){
    motor.setPosition(0);
  }

  public boolean isReached(){
    double rawPos = motor.getRotorPosition().getValueAsDouble();
    double currentPos = rawPos*360;
    double diff = Math.abs(currentPos - revsToMove*360);
    return diff < 5.0;  
  }

  public void moveTo(double rotations){
  revsToMove = rotations * Constants.Extension.GEAR_RATIO;

  // Optional gravity feedforward later
  simAppliedVoltage = 0.0;

  MotionMagicVoltage request = new MotionMagicVoltage(revsToMove);
  motor.setControl(request);
}

  public void stopMotor(){
    motor.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    double error = revsToMove - simRotorPosition;

    double kSimP = 2.0;  
    double voltage = kSimP * error + simAppliedVoltage;

    voltage = Math.max(Math.min(voltage, 12.0), -12.0);

    armSim.setInputVoltage(voltage);
    armSim.update(0.02);

    double armAngleRad = armSim.getAngleRads();
    double armVelocityRadPerSec = armSim.getVelocityRadPerSec();

    simRotorPosition =
        Units.radiansToRotations(armAngleRad) * Constants.Extension.GEAR_RATIO;

    double rotorVelocityRPS =
        Units.radiansToRotations(armVelocityRadPerSec) * Constants.Extension.GEAR_RATIO;

    motorSim.setRawRotorPosition(simRotorPosition);
    motorSim.setRotorVelocity(rotorVelocityRPS);
    motorSim.setSupplyVoltage(voltage);

    arm.setAngle(Units.radiansToDegrees(armAngleRad));

    SmartDashboard.putData("pivot mechanism", mech2d);
    SmartDashboard.putNumber("Motor Voltage", voltage);
    SmartDashboard.putNumber("Rotor Pos", simRotorPosition);
}

  public void periodic() {
    // This method will be called once per scheduler run
  }
}

