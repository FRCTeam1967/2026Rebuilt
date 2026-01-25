// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  
  private TalonFX motor; 
  private TalonFXConfiguration config;
  // private MotionMagicVoltage request;
  private DigitalInput sensor;
  private double rotations;
  private double appliedVoltage;

  // private ProfiledPIDController m_controller;
  // private EncoderSim encoderSim;
  private DIOSim sensorSim;
  private ElevatorSim climbSim; //new ElevatorSim(plant, gearbox, Constants.Climb.MIN_HEIGHT, Constants.Climb.MAX_HEIGHT, true, Constants.Climb.SAFE, 0);
  // private ElevatorFeedforward m_feedforward;
  // private DCMotorSim gearbox;
  private double simRotorPosition;

  private Mechanism2d mechanism;
  private MechanismRoot2d root;
  private MechanismLigament2d tower;
  private MechanismLigament2d carriage;
  private TalonFXSimState motorSim;

  public Climb() {
    motor = new TalonFX(Constants.Climb.MOTOR_ID);
    config = new TalonFXConfiguration();
    sensor = new DigitalInput(Constants.Climb.SENSOR_CHANNEL);
    motorSim = motor.getSimState();


    config.Slot0.kP = Constants.Climb.kP;
    config.Slot0.kI = Constants.Climb.kI;
    config.Slot0.kD = Constants.Climb.kD;
    config.Slot0.kS = Constants.Climb.kS;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Climb.CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = Constants.Climb.ACCELERATION;
    
    config.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Climb.CURRENT_LIMIT));config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);
    
    if (RobotBase.isSimulation()) {
      motorSim = motor.getSimState();
      climbSim =
          new ElevatorSim(
              DCMotor.getKrakenX60(1),
              Constants.Climb.GEAR_RATIO,
              Constants.Climb.CARRIAGE_MASS_KG,
              Constants.Climb.SPROCKET_RADIUS,
              Constants.Climb.MIN_HEIGHT_METERS,
              Constants.Climb.MAX_HEIGHT_METERS,
              true,
              Constants.Climb.SAFE_METERS
          );
      /*
      sensorSim = new DIOSim(sensor);
      mechanism = new Mechanism2d(50,50);
      root = mechanism.getRoot("elevatorRoot", 25, 0);
      tower = root.append(new MechanismLigament2d("tower", 40, 90));
      carriage =
          tower.append(
              new MechanismLigament2d(
                  "carriage", 5, 0, 6, new Color8Bit(Color.kBlue)));
    */
         
      sensorSim = new DIOSim(sensor);

      // Bigger canvas 
      mechanism = new Mechanism2d(120, 90);

      // Put the root near the bottom center
      root = mechanism.getRoot("climbRoot", 60, 8);

      // Two rails (left + right)
      MechanismLigament2d leftRail =
          root.append(new MechanismLigament2d("leftRail", 70, 90, 6, new Color8Bit(Color.kDarkGray)));
      MechanismLigament2d rightRail =
          root.append(new MechanismLigament2d("rightRail", 70, 90, 6, new Color8Bit(Color.kDarkGray)));

      // Offset right rail by adding a short horizontal ligament first
      MechanismRoot2d rightRoot = mechanism.getRoot("rightRoot", 72, 8);
      rightRail =
          rightRoot.append(new MechanismLigament2d("rightRail", 70, 90, 6, new Color8Bit(Color.kDarkGray)));

      // Top crossbar
      MechanismRoot2d topRoot = mechanism.getRoot("topRoot", 60, 78);
      MechanismLigament2d topBar =
          topRoot.append(new MechanismLigament2d("topBar", 28, 0, 6, new Color8Bit(Color.kGray)));

      // Carriage “plate” that moves up/down
      carriage =
          root.append(new MechanismLigament2d("carriageLift", 0, 90, 10, new Color8Bit(Color.kBlue)));

      // Add a plate across the rails (so it looks like a carriage, not a stick)
      MechanismLigament2d plate =
          carriage.append(new MechanismLigament2d("plate", 30, 0, 10, new Color8Bit(Color.kBlue)));

      // a little hook/bumper detail
      plate.append(new MechanismLigament2d("hook", 10, -90, 6, new Color8Bit(Color.kLightBlue)));

    }
  }

  public void simulationInit(){
    motorSim = motor.getSimState();
    motorSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  public void resetEncoders() {
    if (RobotBase.isSimulation()) {
      motorSim.setRawRotorPosition(0);
      simRotorPosition = 0;
    } else {
      motor.setPosition(0);
    }
  }

  public void moveTo(double meters) {  
    rotations = meters*(Constants.Climb.GEAR_RATIO/Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE);
    appliedVoltage = Constants.Climb.FEED_FORWARD;
    MotionMagicVoltage request = new MotionMagicVoltage(rotations).withFeedForward(Constants.Climb.FEED_FORWARD);
    motor.setControl(request);
  }

  public boolean atHeight(){
    double currentPosition = RobotBase.isSimulation() ? simRotorPosition : motor.getRotorPosition().getValueAsDouble();
    return Math.abs(rotations - currentPosition) < Constants.Climb.ERROR_THRESHOLD;
  }

  public void setSafe(){
    if(!sensor.get()){
      motor.setPosition(0);
    }
  }
  public boolean getSensor(){
    return !sensor.get();
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  // public void reachGoal(double goal) {
  //   m_controller.setGoal(goal);
  //   // With the setpoint value we run PID control like normal
  //   double pidOutput = m_controller.calculate(encoderSim.getDistance());
  //   double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
  //   gearbox.setInputVoltage(pidOutput + feedforwardOutput);
  // }
  
  @Override
  public void simulationPeriodic() {
    double error = rotations - simRotorPosition;
    double kSimP = 1.0; 
    double voltage = kSimP * error + appliedVoltage;
    voltage = Math.max(Math.min(voltage, 12.0), -12.0);

    climbSim.setInputVoltage(voltage);
    climbSim.update(0.02);

    simRotorPosition = climbSim.getPositionMeters() / (Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE) * Constants.Climb.GEAR_RATIO;
    motorSim.setRawRotorPosition(simRotorPosition);
    motorSim.setRotorVelocity(climbSim.getVelocityMetersPerSecond() / (Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE) * Constants.Climb.GEAR_RATIO);
    motorSim.setSupplyVoltage(voltage);

    double currentHeight = simRotorPosition / Constants.Climb.GEAR_RATIO * (Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE);
    carriage.setLength(currentHeight);

    boolean atBottom = climbSim.getPositionMeters() <= Constants.Climb.MIN_HEIGHT_METERS + 0.002;
    sensorSim.setValue(!atBottom);

    SmartDashboard.putData("climb mechanism", mechanism);
    SmartDashboard.putBoolean("sensor val", getSensor());
    SmartDashboard.putNumber("position", currentHeight);
    SmartDashboard.putNumber("voltage", voltage);

  }
  
}

