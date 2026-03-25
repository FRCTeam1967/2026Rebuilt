// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
import dev.doglog.DogLog;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

public class Yeeter extends SubsystemBase {
  private TalonFX motor1;
  private TalonFX motor2; //private TalonFX flywheelMotor2;
  private RobotContainer m_robotContainer;
  private Visabelle visabelle;

  private final CANBus canbus = RobotContainer.CANBus;

  private InterpolatingDoubleTreeMap speedTable;

  private Follower followerRequest = new Follower(Constants.Yeeter.YEETER_MOTOR1_ID, MotorAlignmentValue.Opposed);

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
  // private final MechanismRoot2d shooterRoot = shooterMech.getRoot(“shooterRoot”, 1, 1);

  // // A “spoke” that rotates to show the flywheel spinning
  // private final MechanismLigament2d flywheelSpoke =
  //     shooterRoot.append(new MechanismLigament2d(
  //         “flywheelSpoke”,
  //         0.8,  // length
  //         0.0,  // initial angle (deg)
  //         6,
  //         new Color8Bit(Color.kOrange)
  //     ));

  // private double spokeAngleDeg = 0.0;

  /** Creates a new FlywheelShooter. */
  public Yeeter(RobotContainer robotContainer) {
    speedTable = new InterpolatingDoubleTreeMap();
    motor1 = new TalonFX(Constants.Yeeter.YEETER_MOTOR1_ID, canbus);
    motor2 = new TalonFX(Constants.Yeeter.YEETER_MOTOR2_ID, canbus);
    // flywheelMotor2 = new Follower(Constants.FlywheelShooter.FLYWHEELSHOOTER_MOTOR1_ID, MotorAlignmentValue.Opposed);

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = Constants.Yeeter.kS;
    slot0Configs.kV = Constants.Yeeter.kV;
    slot0Configs.kA = Constants.Yeeter.kA;
    slot0Configs.kP = Constants.Yeeter.kP;
    slot0Configs.kI = Constants.Yeeter.kI;
    slot0Configs.kD = Constants.Yeeter.kD;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Yeeter.CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Yeeter.ACCELERATION;
    //motionMagicConfigs.MotionMagicJerk = Constants.FlywheelShooter.JERK;

    talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    motor1.setNeutralMode(NeutralModeValue.Coast);
    motor2.setNeutralMode(NeutralModeValue.Coast);

    m_robotContainer = robotContainer;
    visabelle = m_robotContainer.visabelle;

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    //SmartDashboard.putData(“ShooterMech2d”, shooterMech);


    motor2.setControl(followerRequest);

    motor1.getConfigurator().apply(talonFXConfigs);
    motor2.getConfigurator().apply(talonFXConfigs);
    populateTreeMap();
  }

  /**
   * set velocity of the motor with MotionMagicVelocityVoltage requests params </p>
   * creates torque request but doesn't use it?
   * @param velocity
   * @param acceleration
   */
  public void setVelocity(DoubleSupplier velocity, double acceleration) {
    MotionMagicVelocityVoltage requestOne = new MotionMagicVelocityVoltage(-velocity.getAsDouble())
      .withAcceleration(acceleration);
      //.withFeedForward(5.0);

    // MotionMagicVelocityTorqueCurrentFOC torqueRequest = new MotionMagicVelocityTorqueCurrentFOC(velocity)
    //   .withFeedForward(0.12); //kS value ?

    // MotionMagicVelocityVoltage requestTwo = new MotionMagicVelocityVoltage(-velocity)
    //   .withAcceleration(acceleration);
    // //   //.withFeedForward(5.0);

    motor1.setControl(requestOne);
    motor2.setControl(followerRequest); //TODO: this is what was in Sunday's code. should we be setting two different control requests on the same motor?
  }

  /**
   * @return true if current speed of yeeter is >= threshold speed
   */
  public boolean reachedYeeterSpeed() {
    return (Math.abs(motor1.getVelocity().getValueAsDouble()) >= (getNecessarySpeed(() -> visabelle.getDisFromHub())));
  }

  /**
   * @return average velocity of both motors
   */
  public double getCurrentVelocity() {
    double currentVelocity1 = motor1.getVelocity().getValueAsDouble();
    double currentVelocity2 = Math.abs(motor2.getVelocity().getValueAsDouble());
    double averageVelocity = (currentVelocity1 + currentVelocity2)/2;
    return(averageVelocity);
  }

  /**
   * stops both motors, obviously :)
   */
  public void stopMotor() {
    motor1.stopMotor();
    motor2.setControl(followerRequest);
  }

  /**
   * @param motor
   * @return velocity as double of motor
   */
  public double getMotorVelocity(TalonFX motor) {
    return (motor.getVelocity().getValueAsDouble());
  }

  public void configDashboard(ShuffleboardTab tab) {
    //tab.addDouble(“FlywheelSpeed”, () -> getMotorVelocity(flywheelMotor1));
    tab.addDouble("YeeterSpeed1", () -> getMotorVelocity(motor1));
    tab.addDouble("YeeterSpeed2", () -> getMotorVelocity(motor2));
    //tab.addDouble("TargetVelocity", Constants.Yeeter.YEETER_SPEED);
  }

  //TODO: fill in these values
  /**
   * populates angle tables with given distance and hood angles
   */
  private void populateTreeMap() {
    //distance from hub (m), shooter speeds
    speedTable.put(0.6858+1.02235, 50.0); //2ft
    speedTable.put(1.524+1.02235, 64.5); //5ft
    speedTable.put(1.8288+1.02235, 66.7); //6ft
    speedTable.put(2.4384+1.02235, 72.0); //8ft
    speedTable.put(3.048+1.02235, 74.0); //10ft

    // speedTable.put(3.3288, 68.0); //6 feet
    // speedTable.put(3.9384, 75.0); //8 feet
  }

  /**
   * @param distanceToHub
   * @return speed of the shooter based on distance in tree map
   */
  //TODO: call this in robot container when setting speed
  public double getNecessarySpeed(DoubleSupplier distanceToHub) {
    double speed = speedTable.get(distanceToHub.getAsDouble());
    DogLog.log("target", speed);
    return speed;
  }

  public void logYeeterSpeeds(){
    DogLog.log("YeeterSpeed1", getMotorVelocity(motor1));
    DogLog.log("YeeterSpeed2", getMotorVelocity(motor2));
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
    //   SmartDashboard.putNumber(“Flywheel/RotorRPS”, 0.0);
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

    // SmartDashboard.putNumber(“Flywheel/RotorRPS”, rotorRps);
    // SmartDashboard.putNumber(“Flywheel/MotorVolts”, motorVolts);
    // SmartDashboard.putNumber(“Flywheel/SpokeAngleDeg”, spokeAngleDeg);
  }
}