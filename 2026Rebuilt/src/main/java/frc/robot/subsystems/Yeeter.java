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
import edu.wpi.first.networktables.DoubleSubscriber;
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
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

public class Yeeter extends SubsystemBase {
  private TalonFX motor1;
  private TalonFX motor2;
  private final RobotContainer m_robotContainer; 

  //private TalonFX flywheelMotor2;
  //private Visabelle visabelle;

  private final CANBus canbus = RobotContainer.CANBus;

  private InterpolatingDoubleTreeMap speedTable;

  private final DoubleSubscriber yeeterAcceleration = DogLog.tunable("Yeeter/yeeterAcceleration", Constants.Yeeter.YEETER_ACCELERATION);
  private final DoubleSubscriber cruiseVelocity = DogLog.tunable("Yeeter/cruiseVelocity", Constants.Yeeter.CRUISE_VELOCITY);
  private final DoubleSubscriber mmAcceleration = DogLog.tunable("Yeeter/mmAcceleration", Constants.Yeeter.ACCELERATION);
  //private final DoubleSubscriber feedForward = DogLog.tunable("Yeeter/feedForward", 5.0);

  private Follower followerRequest = new Follower(Constants.Yeeter.YEETER_MOTOR1_ID, MotorAlignmentValue.Opposed);

  /** Creates a new FlywheelShooter. */
  public Yeeter(RobotContainer robotContainer){//Visabelle visabelle) {
    speedTable = new InterpolatingDoubleTreeMap();
    motor1 = new TalonFX(Constants.Yeeter.YEETER_MOTOR1_ID, canbus);
    motor2 = new TalonFX(Constants.Yeeter.YEETER_MOTOR2_ID, canbus);
    m_robotContainer = robotContainer; 

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    // slot0Configs.kS = Constants.Yeeter.kS;
    // slot0Configs.kV = Constants.Yeeter.kV;
    // slot0Configs.kA = Constants.Yeeter.kA;
    // slot0Configs.kP = Constants.Yeeter.kP;
    // slot0Configs.kI = Constants.Yeeter.kI;
    // slot0Configs.kD = Constants.Yeeter.kD;

    DogLog.tunable("Yeeter/kP", slot0Configs.kP, 
      newP -> {
        motor1.getConfigurator().apply(slot0Configs.withKP(newP));
        motor2.setControl(followerRequest);
      }
    );

    DogLog.tunable("Yeeter/kI", slot0Configs.kI, 
      newI -> {
        motor1.getConfigurator().apply(slot0Configs.withKI(newI));
        motor2.setControl(followerRequest);
      }
    );

    DogLog.tunable("Yeeter/kD", slot0Configs.kD, 
      newD -> {
        motor1.getConfigurator().apply(slot0Configs.withKD(newD));
        motor2.setControl(followerRequest);
      }
    );

    DogLog.tunable("Yeeter/kS", slot0Configs.kS, 
      newS -> {
        motor1.getConfigurator().apply(slot0Configs.withKS(newS));
        motor2.setControl(followerRequest);
      }
    );

    DogLog.tunable("Yeeter/kV", slot0Configs.kV, 
      newV -> {
        motor1.getConfigurator().apply(slot0Configs.withKV(newV));
        motor2.setControl(followerRequest);
      }
    );

    DogLog.tunable("Yeeter Speed", Constants.Yeeter.YEETER_SPEED);

    slot0Configs.kA = Constants.Yeeter.kA;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Yeeter.CRUISE_VELOCITY;
    // motionMagicConfigs.MotionMagicAcceleration = Constants.Yeeter.ACCELERATION;

    motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity.get();
    motionMagicConfigs.MotionMagicAcceleration = mmAcceleration.get();

    //motionMagicConfigs.MotionMagicJerk = Constants.FlywheelShooter.JERK;

    talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    motor1.setNeutralMode(NeutralModeValue.Coast);
    motor2.setNeutralMode(NeutralModeValue.Coast);

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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
  public void setVelocity(DoubleSupplier velocitySupplier, double acceleration) {
    // velocitySupplier should probably stored as a member field and then used in reachedYeeterSpeed() rather than 
    // having reachedYeeterSpeed() reach into robot container to get the Visabelle. You'd probably want to have
    // stopMotor() then set velocitySupplier to NULL or set it to a DoubleSupplier that returns a constant value of 0.

    double velocity = velocitySupplier.getAsDouble();

    // MotionMagicVelocityVoltage requestOne = new MotionMagicVelocityVoltage(-velocity)
    //   .withAcceleration(yeeterAcceleration.get());
      //.withFeedForward(5.0);
      //.withFeedFoward(feedForward.get());

    MotionMagicVelocityTorqueCurrentFOC torqueRequest = new MotionMagicVelocityTorqueCurrentFOC(-velocity)
    .withAcceleration(yeeterAcceleration.get()) ; 
    //.withFeedForward(0.12); //kS value ?

    // MotionMagicVelocityVoltage requestTwo = new MotionMagicVelocityVoltage(-velocity)
    //   .withAcceleration(acceleration);
    // //   //.withFeedForward(5.0);

    DogLog.log("Yeeter/target speed (set)", -velocity);

    motor1.setControl(torqueRequest);
    motor2.setControl(followerRequest); //TODO: this is what was in Sunday's code. should we be setting two different control requests on the same motor?
  }

  /**
   * @return true if current speed of yeeter is >= threshold speed
   */
  public boolean reachedYeeterSpeed() {
    // Since we're using a double supplier, the value we get here may be different than the value we got in setVelocity().
    double necessarySpeed = getNecessarySpeed(() -> m_robotContainer.visabelle.getDisFromHub());
    DogLog.log("Yeeter/target speed (is reached)", necessarySpeed);
    return (Math.abs(motor1.getVelocity().getValueAsDouble()) >= necessarySpeed);
    //return (Math.abs(motor1.getVelocity().getValueAsDouble()) >= (getNecessarySpeed(() -> m_robotContainer.visabelle.getDisFromHub())));
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
    DogLog.log("Yeeter/target speed", 0);
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

  // public void configDashboard(ShuffleboardTab tab) {
  //   //tab.addDouble(“FlywheelSpeed”, () -> getMotorVelocity(flywheelMotor1));
  //   // tab.addDouble("YeeterSpeed1", () -> getMotorVelocity(motor1));
  //   // tab.addDouble("YeeterSpeed2", () -> getMotorVelocity(motor2));
  //   //tab.addDouble("TargetVelocity", Constants.Yeeter.YEETER_SPEED);
  // }

  //TODO: fill in these values
  /**
   * populates angle tables with given distance and hood angles
   */
  private void populateTreeMap() {
    //distance from hub (m), shooter speeds
    speedTable.put(1.524+1.02235, 64.0); //5ft //TESTED
    speedTable.put(1.676+1.02235, 65.0); //5.5ft //TESTED
    speedTable.put(1.8288+1.02235, 66.0); //6ft //TESTED
    speedTable.put(2.1366+1.02235, 67.0);//7ft //TESTED
    speedTable.put(2.286+1.02235, 69.0); //7.5 ftUNTESTED
    speedTable.put(2.4384+1.02235, 69.5); //8ft //78
    // speedTable.put(3.353+1.02235, 79.0); //11ft // change speed

    // speedTable.put(3.3288, 68.0); //6 feet
    // speedTable.put(3.9384, 75.0); //8 feet
  }

  /**
   * @param distanceToHub
   * @return speed of the shooter based on distance in tree map
   */
  //TODO: call this in robot container when setting speed
  public double getNecessarySpeed(DoubleSupplier distanceToHub) {
    double distance = distanceToHub.getAsDouble();
    double speed = speedTable.get(distance);
    DogLog.log("Yeeter/distance", distance);
    DogLog.log("Yeeter/target", speed);
    return speed;
  }

  @Override
  public void periodic() {
    DogLog.log("Yeeter/Speed1", getMotorVelocity(motor1));
    DogLog.log("Yeeter/Speed2", getMotorVelocity(motor2));

    if (Constants.Yeeter.verboseLogging) {
      DogLog.log("Yeeter/stator current 1", motor1.getStatorCurrent().getValueAsDouble());
      DogLog.log("Yeeter/stator current 2", motor2.getStatorCurrent().getValueAsDouble());
      DogLog.log("Yeeter/reached speed?", reachedYeeterSpeed());
    }
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
  }
}