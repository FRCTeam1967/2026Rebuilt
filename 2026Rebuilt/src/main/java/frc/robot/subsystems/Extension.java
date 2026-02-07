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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;


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
  private PIDController controller;
  private Field2d field;
  private DifferentialDrivetrainSim swerve;
    private double rotations;
  private double appliedVoltage;
  private Pose3d poses;
  private Rotation3d rotation;
  /** Creates a new Pivot. */
  public Extension() {
    motor = new TalonFX (Constants.Extension.EXTENSION_MOTOR_ID);
    motorSim = motor.getSimState();
    field = new Field2d();
    swerve = new DifferentialDrivetrainSim(null, null, simRotorPosition, rotations, appliedVoltage, null);
    rotation = new Rotation3d();
    poses = new Pose3d(0.0,0.0,0.0,rotation);
    controller = new PIDController(20, 6, 6);
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
    Pose3d poseA = new Pose3d();
    Pose3d poseB = new Pose3d();

    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose3d.struct).publish();
    StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();
  }

  public void simulationInit(){
    motorSim = motor.getSimState();
    motorSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

    
  public void resetEncoders(){
    motor.setPosition(0);
  }

  public boolean isReached(){
    double currentPos = motor.getPosition().getValueAsDouble();
    double diff = Math.abs(currentPos - revsToMove);
    return diff < 0.02; 
  }

  public void moveTo(double rotations){
  revsToMove = rotations;

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

    // double voltage = kSimP * error + simAppliedVoltage;
    double voltage = controller.calculate(simRotorPosition, revsToMove);

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
    field.setRobotPose(swerve.getPose());

    SmartDashboard.putData("pivot mechanism", mech2d);
    SmartDashboard.putNumber("Motor Voltage", voltage);
    SmartDashboard.putNumber("Rotor Pos", simRotorPosition);

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

  public void periodic() {
    // This method will be called once per scheduler run
  }
}

