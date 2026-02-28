// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N2;
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.simulation.DIOSim;
// import edu.wpi.first.wpilibj.simulation.ElevatorSim;
// import edu.wpi.first.wpilibj.simulation.EncoderSim;
// import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// import java.util.function.DoubleSupplier;

// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.sim.TalonFXSimState;

// public class Climb extends SubsystemBase {
//   /** Creates a new Climb. */
  
//   private TalonFX motor; 
//   private TalonFXConfiguration config;
//   // private MotionMagicVoltage request;
//   private DigitalInput sensor;
//   private double rotations;
 

//   private TalonFXSimState motorSim;
//   // private ProfiledPIDController m_controller;
//   // private EncoderSim encoderSim;
//   private DIOSim sensorSim;
//   private ElevatorSim climbSim; //new ElevatorSim(plant, gearbox, Constants.Climb.MIN_HEIGHT, Constants.Climb.MAX_HEIGHT, true, Constants.Climb.SAFE, 0);
//   // private ElevatorFeedforward m_feedforward;
//   // private DCMotorSim gearbox;

//   private Mechanism2d mechanism;
//   private MechanismRoot2d root;
//   private MechanismLigament2d tower;
//   private MechanismLigament2d carriage;

//   public Climb() {
//     motor = new TalonFX(Constants.Climb.MOTOR_ID);
//     config = new TalonFXConfiguration();
//     sensor = new DigitalInput(Constants.Climb.SENSOR_CHANNEL);

//     config.Slot0.kP = Constants.Climb.kP;
//     config.Slot0.kI = Constants.Climb.kI;
//     config.Slot0.kD = Constants.Climb.kD;
//     config.Slot0.kS = Constants.Climb.kS;

//     config.MotionMagic.MotionMagicCruiseVelocity = Constants.Climb.CRUISE_VELOCITY;
//     config.MotionMagic.MotionMagicAcceleration = Constants.Climb.ACCELERATION;
    
//     config.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Climb.CURRENT_LIMIT));
    
//     motor.getConfigurator().apply(config);
    
//     if (RobotBase.isSimulation()) {
//       motorSim = motor.getSimState();
//       climbSim =
//           new ElevatorSim(
//               DCMotor.getFalcon500(1),
//               Constants.Climb.GEAR_RATIO,
//               Constants.Climb.CARRIAGE_MASS_KG,
//               Constants.Climb.SPROCKET_RADIUS,
//               Constants.Climb.MIN_HEIGHT,
//               Constants.Climb.MAX_HEIGHT,
//               true,
//               Constants.Climb.SAFE
//           );

//       sensorSim = new DIOSim(sensor);
//       mechanism = new Mechanism2d(50,50);
//       root = mechanism.getRoot("elevatorRoot", 25, 0);
//       tower = root.append(new MechanismLigament2d("tower", 40, 90));
//       carriage =
//           tower.append(
//               new MechanismLigament2d(
//                   "carriage", 5, 0, 6, new Color8Bit(Color.kBlue)));


//     }
//   }

//   public void resetEncoders() {
//      motor.setPosition(0);
//   }

//   public void moveTo(double inches) {  
//     rotations = inches*(Constants.Climb.GEAR_RATIO/Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE);
//     MotionMagicVoltage request = new MotionMagicVoltage(rotations).withFeedForward(Constants.Climb.FEED_FORWARD);
//     motor.setControl(request);
//   }

//   public boolean atHeight(){
//     double currentPosition = motor.getRotorPosition().getValueAsDouble();
//     double targetPosition = rotations;
//     double error = Math.abs(targetPosition-currentPosition);
//     return(error<Constants.Climb.ERROR_THRESHOLD);
//   }

//   public void setSafe(){
//     if(!sensor.get()){
//       motor.setPosition(0);
//     }
//   }
//   public boolean getSensor(){
//     return !sensor.get();
//   }

//   public void stopMotor() {
//     motor.stopMotor();
//   }

//   // public void reachGoal(double goal) {
//   //   m_controller.setGoal(goal);
//   //   // With the setpoint value we run PID control like normal
//   //   double pidOutput = m_controller.calculate(encoderSim.getDistance());
//   //   double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
//   //   gearbox.setInputVoltage(pidOutput + feedforwardOutput);
//   // }
  
//   @Override
//   public void simulationPeriodic() {
//     motorSim.setSupplyVoltage(12.0);

//     climbSim.setInputVoltage(motorSim.getMotorVoltage());
//     climbSim.update(0.02);

//     double heightMeters = climbSim.getPositionMeters();

//     double rotorRotations =
//         heightMeters
//             / (Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE * 0.0254)
//             * Constants.Climb.GEAR_RATIO;

//     motorSim.setRawRotorPosition(rotorRotations);

//     motorSim.setRotorVelocity(
//         climbSim.getVelocityMetersPerSecond()
//             / (Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE * 0.0254)
//             * Constants.Climb.GEAR_RATIO
//     );
//     double currentHeight =
//       motor.getRotorPosition().getValueAsDouble()
//           / Constants.Climb.GEAR_RATIO
//           * (Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE * 0.0254);

//     carriage.setLength(currentHeight);

//     boolean atBottom =
//         heightMeters <= Constants.Climb.MIN_HEIGHT + 0.002;

//     sensorSim.setValue(!atBottom); 

//     SmartDashboard.putData("climb mechanism", mechanism);
//     SmartDashboard.putBoolean("sensor val", getSensor());
//     SmartDashboard.putNumber("position", motor.getRotorPosition().getValueAsDouble()/ Constants.Climb.GEAR_RATIO*(Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE * 0.0254));
//     SmartDashboard.putNumber("voltage", motorSim.getMotorVoltage());

//   }

  
// }

