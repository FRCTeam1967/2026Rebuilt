// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class FlywheelShooter extends SubsystemBase {
  private TalonFX flywheelShooterMotor;

  private TalonFXConfiguration config;

  /** Creates a new Shooter. */
  public FlywheelShooter() {
    flywheelShooterMotor = new TalonFX(Constants.FlywheelShooter.FLYWHEELSHOOTER_MOTOR_ID);

    config = new TalonFXConfiguration();

    var talonFXConfigs = new TalonFXConfiguration();

    // Slot0 configs
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = Constants.FlywheelShooter.kS;
    slot0Configs.kV = Constants.FlywheelShooter.kV;
    slot0Configs.kA = Constants.FlywheelShooter.kA;
    slot0Configs.kP = Constants.FlywheelShooter.kP;
    slot0Configs.kI = Constants.FlywheelShooter.kI;
    slot0Configs.kD = Constants.FlywheelShooter.kD;

    // Motion Magic configs
    //var motionMagicConfigs = talonFXConfigs.MotionMagic;
    //motionMagicConfigs.MotionMagicCruiseVelocity = Constants.FlywheelShooter.CRUISE_VELOCITY;
    //motionMagicConfigs.MotionMagicAcceleration = Constants.FlywheelShooter.ACCELERATION;
    //motionMagicConfigs.MotionMagicJerk = Constants.FlywheelShooter.JERK;

    //talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  
    //talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //applies motion magic configs
    //flywheelShooterMotor.getConfigurator().apply(talonFXConfigs);

    // Apply configs
    flywheelShooterMotor.getConfigurator().apply(talonFXConfigs);

   flywheelShooterMotor.setNeutralMode(NeutralModeValue.Brake);

    // Zero if switch is already pressed at boot
  }
  
  //CHANGE AND REMOVE THESE METHODS FOR SHOOTER
  public void stop() {
      flywheelShooterMotor.stopMotor();
  }

  public void runMotor(double flywheelAccelration, double flywheelVelocity){
    
  }

  /**Adds values to shuffleboard 
  * we don't know if we need shuffle board values for right now on shooter but this will be here just in case
  */
  public void configDashboard(ShuffleboardTab tab) {

  }

    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
   

}
