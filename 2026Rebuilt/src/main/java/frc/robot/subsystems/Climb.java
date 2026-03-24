// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import dev.doglog.DogLog;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;



public class Climb extends SubsystemBase {  
  private TalonFX motor; 
  private TalonFXConfiguration config;
  private DigitalInput bottomSensor;
  private DigitalInput topSensor;
  private double rotations;
  private double appliedVoltage;
  private MotionMagicVoltage request;
  private final CANBus canbus = RobotContainer.CANBus;

  // SIM FIELDS
  // private ProfiledPIDController m_controller;
  // private EncoderSim encoderSim;
  private DifferentialDrivetrainSim swerve;
  private DIOSim sensorSim;
  private ElevatorSim climbSim; //new ElevatorSim(plant, gearbox, Constants.Climb.MIN_HEIGHT, Constants.Climb.MAX_HEIGHT, true, Constants.Climb.SAFE, 0);
  private ElevatorFeedforward m_feedforward;
  private DCMotorSim gearbox;
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
    bottomSensor = new DigitalInput(Constants.Climb.BOTTOM_SENSOR_CHANNEL);
    topSensor = new DigitalInput(Constants.Climb.TOP_SENSOR_CHANNEL);
    request = new MotionMagicVoltage(rotations).withFeedForward(Constants.Climb.FEED_FORWARD);


    // SIM INITS
    if (RobotBase.isSimulation()){
      motorSim = motor.getSimState();
      field = new Field2d();
      swerve = new DifferentialDrivetrainSim(null, null, simRotorPosition, rotations, appliedVoltage, null);
      rotation = new Rotation3d();
      poses = new Pose3d(0.0,0.0,0.0,rotation);
    }

    CANcoderConfiguration ccdConfigs = new CANcoderConfiguration();


    ccdConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    // var limitConfigs = new CurrentLimitsConfigs();
    // limitConfigs.StatorCurrentLimit = 1;
    // limitConfigs.StatorCurrentLimitEnable = true;
    
    config.Slot0.kP = Constants.Climb.kP;
    config.Slot0.kI = Constants.Climb.kI;
    config.Slot0.kD = Constants.Climb.kD;
    config.Slot0.kS = Constants.Climb.kS;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Climb.CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = Constants.Climb.ACCELERATION;
    
    config.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Climb.CURRENT_LIMIT));config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);
    
  }
    /* 
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
    /*     
      sensorSim = new DIOSim(bottomSensor);

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
              "plate", 0.6, 0, t, new Color8Bit(Color.kBlue)));

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
*/

/* 
  public void simulationInit(){
    motorSim = motor.getSimState();
    motorSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }
*/
  /**
   * set existing position of motors to be 0
   */
  public void resetEncoders() {
    if (RobotBase.isSimulation()) {
      motorSim.setRawRotorPosition(0);
      simRotorPosition = 0;
    } else {
      motor.setPosition(0);
    }
  }

  /**
   * @param inches - converted to rotations </p>
   * sets appliedVoltage to feedforward </p>
   * creates and sets a MotionMagicVoltage request with rotations and feedforward
   */
  public void moveTo(double inches) {  
    rotations = inches*(Constants.Climb.GEAR_RATIO/Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE);
    appliedVoltage = Constants.Climb.FEED_FORWARD;
    motor.setControl(request.withPosition(rotations));
  }

  /**
   * @return true if motor's current position is within error threshold of target height
   */
  public boolean atHeight(){
    double currentPosition = motor.getRotorPosition().getValueAsDouble();
    return Math.abs(rotations - currentPosition) < Constants.Climb.ERROR_THRESHOLD;

    /*
    if (getBottomSensor() || getTopSensor()){
      return true;
    }
    return false; */
  }

  /**
   * TODO: write comments for climb once we figure out 
   */
  public void setSafe(){
    if(getBottomSensor()){
      motor.setPosition(0);
    }
  }

  public boolean getBottomSensor(){
    return !bottomSensor.get();
  }
    public boolean getTopSensor(){
    return !topSensor.get();
  }

  public boolean isReachedTopSwitch(){
    return (getTopSensor());
  }

  public boolean isReachedBottomSwitch(){
    return (getBottomSensor());
  }

  /**
   * Stops motor
   */
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
  public void periodic() {
    double rotorPosition = motor.getRotorPosition().getValueAsDouble();
    DogLog.log("Climb/at height", Math.abs(rotations) - Math.abs(rotorPosition) < Constants.Climb.ERROR_THRESHOLD);
    DogLog.log("Climb/target rotations", rotations);
    DogLog.log("Climb/rotations", rotorPosition);
    DogLog.log("Climb/inches", rotorPosition/(Constants.Climb.GEAR_RATIO/Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE));
    DogLog.log("Climb/bottom sensor", getBottomSensor());
    DogLog.log("Climb/top sensor", getTopSensor());

    if (Constants.Climb.verboseLogging) {
      DogLog.log("Climb/stator current", motor.getStatorCurrent().getValueAsDouble());
    }

    //setSafe();
  }
  /*
   * @Override
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
  */
  
  
}

