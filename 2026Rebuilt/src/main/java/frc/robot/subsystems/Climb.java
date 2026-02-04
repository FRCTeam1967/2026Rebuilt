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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
  private DifferentialDrivetrainSim swerve;

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
  private Field2d field;
  
  private Pose3d poses;
  private Rotation3d rotation;

  public Climb() {
    motor = new TalonFX(Constants.Climb.MOTOR_ID);
    config = new TalonFXConfiguration();
    sensor = new DigitalInput(Constants.Climb.SENSOR_CHANNEL);
    motorSim = motor.getSimState();
    field = new Field2d();
    swerve = new DifferentialDrivetrainSim(null, null, simRotorPosition, rotations, appliedVoltage, null);
    rotation = new Rotation3d();
    poses = new Pose3d(0.0,0.0,0.0,rotation);
    

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

      mechanism = new Mechanism2d(2, 2);

      // bottom center
      root = mechanism.getRoot("climbRoot", 1.0, 0.2);

      // uniform thickness for everything
      double t = 1;  // thicker than before

      // rails
      MechanismLigament2d leftRail =
          root.append(new MechanismLigament2d(
              "leftRail", 1.0, 90, t, new Color8Bit(Color.kDarkGray)));

      MechanismRoot2d rightRoot = mechanism.getRoot("rightRoot", 1.3, 0.2);
      MechanismLigament2d rightRail =
          rightRoot.append(new MechanismLigament2d(
              "rightRail", 1.0, 90, t, new Color8Bit(Color.kDarkGray)));

      // carriage
      carriage =
          root.append(new MechanismLigament2d(
              "carriageLift", 0.0, 90, t, new Color8Bit(Color.kBlue)));

      // plate
      MechanismLigament2d plate =
          carriage.append(new MechanismLigament2d(
              "plate", 0.6, 0, t, new Color8Bit(Color.kBlue))); //length:0.6

      // hook
      plate.append(new MechanismLigament2d(
          "hook", 0.30, -90, t, new Color8Bit(Color.kLightBlue)));//length: 0.25
    
    Pose3d poseA = new Pose3d();
    Pose3d poseB = new Pose3d();

    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose3d.struct).publish();
    StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();
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
    field.setRobotPose(swerve.getPose());

    SmartDashboard.putData("climb mechanism", mechanism);
    // SmartDashboard.putData("climb mechanism", mechanism.generate3dMechanism());
    SmartDashboard.putBoolean("sensor val", getSensor());
    SmartDashboard.putNumber("position", currentHeight);
    SmartDashboard.putNumber("voltage", voltage);
    /*
    SmartDashboard.putNumber("pose3d x", poses.getX());
    SmartDashboard.putNumber("pose3d y", poses.getY());
    SmartDashboard.putNumber("pose3d z", poses.getZ());
    SmartDashboard.putNumber("pose3d rotx", poses.getRotation().getX());
    SmartDashboard.putNumber("pose3d roty", poses.getRotation().getY());
    SmartDashboard.putNumber("pose3d rotz", poses.getRotation().getZ());
    */
    SmartDashboard.putNumberArray("robotPose", new double[] {
      poses.getX(),
      poses.getY(),
      poses.getZ(),
      poses.getRotation().getX(),
      poses.getRotation().getY(),
      poses.getRotation().getZ()
    });
    SmartDashboard.putData("field", field);
    

  }
  
}

