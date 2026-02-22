// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volts;

public class FlywheelShooter extends SubsystemBase {

  private TalonFX flywheelMotor1;
  private TalonFX flywheelMotor2;

  private final CANBus canbus = new CANBus("CANivore");

  private boolean reachedShooterSpeed = false;


  // private static final double kSimDt = 0.02;
  // private double simRotorPosRot = 0.0;


  // private final DCMotorSim flywheelSim = new DCMotorSim(
  //     LinearSystemId.createDCMotorSystem(
  //         DCMotor.getKrakenX60Foc(1),
  //         0.001,
  //         Constants.FlywheelShooter.GEAR_RATIO
  //     ),
  //     DCMotor.getKrakenX60Foc(1)
  // );
  // private final Mechanism2d shooterMech = new Mechanism2d(2, 2);
  // private final MechanismRoot2d shooterRoot = shooterMech.getRoot("shooterRoot", 1, 1);

  // // A "spoke" that rotates to show the flywheel spinning
  // private final MechanismLigament2d flywheelSpoke =
  //     shooterRoot.append(new MechanismLigament2d(
  //         "flywheelSpoke",
  //         0.8,  // length
  //         0.0,  // initial angle (deg)
  //         6,    
  //         new Color8Bit(Color.kOrange)
  //     ));

  // private double spokeAngleDeg = 0.0;

  /** Creates a new FlywheelShooter. */
  public FlywheelShooter() {
    flywheelMotor1 = new TalonFX(Constants.FlywheelShooter.FLYWHEELSHOOTER_MOTOR1_ID, canbus);
    flywheelMotor2 = new TalonFX(Constants.FlywheelShooter.FLYWHEELSHOOTER_MOTOR2_ID, canbus);

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
//motionMagicConfigs.MotionMagicJerk = Constants.FlywheelShooter.JERK;

    talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    flywheelMotor1.setNeutralMode(NeutralModeValue.Brake);
    flywheelMotor2.setNeutralMode(NeutralModeValue.Brake);

    flywheelMotor1.getConfigurator().apply(talonFXConfigs);
    flywheelMotor2.getConfigurator().apply(talonFXConfigs);

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    reachedShooterSpeed = false;
    //SmartDashboard.putData("ShooterMech2d", shooterMech);
  }

  public void setVelocity(double velocity, double acceleration) { //set the velocity of the shooter
    MotionMagicVelocityVoltage requestOne = new MotionMagicVelocityVoltage(velocity)
      .withAcceleration(acceleration);
      //.withFeedForward(5.0);

    MotionMagicVelocityVoltage requestTwo = new MotionMagicVelocityVoltage(-velocity)
      .withAcceleration(acceleration);
      //.withFeedForward(5.0);

    flywheelMotor1.setControl(requestOne);
    flywheelMotor2.setControl(requestTwo);
  }

  public boolean reachedShooterSpeed() { //checks if the shooter has reached the target speed
    return (getAverageVelocity() >= Constants.FlywheelShooter.SHOOTER_THRESHOLD_SPEED1);
  }

  public double getAverageVelocity() { //checks if the shooter has reached the target speed
  
    double currentVelocity1 = flywheelMotor1.getVelocity().getValueAsDouble();
    double currentVelocity2 = Math.abs(flywheelMotor2.getVelocity().getValueAsDouble());
    double averageVelocity = (currentVelocity1 + currentVelocity2)/2;
    return(averageVelocity);

  }

  public void stopMotor() { //stops the hood motor (obviously : ))
    flywheelMotor1.stopMotor();
    flywheelMotor2.stopMotor();
  }

  public double getMotorVelocity(TalonFX motor) { //stops the hood motor (obviously : ))
    return (motor.getVelocity().getValueAsDouble());
  }

  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Flywheel Left Speed", () -> getMotorVelocity(flywheelMotor1));
    tab.addDouble("Flywheel Right Speed", () -> getMotorVelocity(flywheelMotor2));
    tab.addDouble("Flywheel Average Velocity", () -> getAverageVelocity());
  }

  @Override
  public void periodic() {    
  }

  @Override
  public void simulationPeriodic() {

    
    // if (RobotState.isDisabled()) {
    //   stopMotor();


    //   flywheelSim.setInputVoltage(0.0);
    //   flywheelSim.update(kSimDt);

    //   simRotorPosRot = 0.0;

    //   var sim1 = flywheelMotor1.getSimState();
    //   var sim2 = flywheelMotor2.getSimState();

    //   double batteryV = RobotController.getBatteryVoltage();
    //   sim1.setSupplyVoltage(batteryV);
    //   sim2.setSupplyVoltage(batteryV);

    //   sim1.setRotorVelocity(0.0);
    //   sim2.setRotorVelocity(0.0);

    //   sim1.setRawRotorPosition(0.0);
    //   sim2.setRawRotorPosition(0.0);

    //   flywheelSpoke.setAngle(0.0);
    //   SmartDashboard.putNumber("Flywheel/RotorRPS", 0.0);
    //   return;
    // }

    // var sim1 = flywheelMotor1.getSimState();
    // var sim2 = flywheelMotor2.getSimState();

    // double batteryV = RobotController.getBatteryVoltage();
    // sim1.setSupplyVoltage(batteryV);
    // sim2.setSupplyVoltage(batteryV);

    // double motorVolts = sim1.getMotorVoltageMeasure().in(Volts);

    // flywheelSim.setInputVoltage(motorVolts);
    // flywheelSim.update(kSimDt);

    // double mechRps = flywheelSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    // double rotorRps = mechRps * Constants.FlywheelShooter.GEAR_RATIO;

    // simRotorPosRot += rotorRps * kSimDt;

    // sim1.setRotorVelocity(rotorRps);
    // sim2.setRotorVelocity(rotorRps*3);

    // sim1.setRawRotorPosition(simRotorPosRot);
    // sim2.setRawRotorPosition(simRotorPosRot);


    // spokeAngleDeg = (spokeAngleDeg + rotorRps * 360.0 * kSimDt) % 360.0;
    // flywheelSpoke.setAngle(spokeAngleDeg);

    // SmartDashboard.putNumber("Flywheel/RotorRPS", rotorRps);
    // SmartDashboard.putNumber("Flywheel/MotorVolts", motorVolts);
    // SmartDashboard.putNumber("Flywheel/SpokeAngleDeg", spokeAngleDeg);
  }
}
