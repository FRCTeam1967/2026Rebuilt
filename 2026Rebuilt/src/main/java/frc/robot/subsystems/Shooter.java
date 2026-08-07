// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX motor1; // Declare motor 1
  private TalonFX _________; // Declare motor 2

  /** Creates a new Shooter. */
  public Shooter() {
    motor1 = new TalonFX(Constants.Shooter.FIRST_SHOOTER_ID); // Set the first shooter ID

    // Initialize the bottom motor in the same way:
    _________ = new TalonFX(______); // Use the SECOND_SHOOTER_ID value here

    //Configurations
    var talonFXConfigs = new TalonFXConfiguration();

    motor1.getConfigurator().apply(talonFXConfigs);
    motor2.getConfigurator().apply(talonFXConfigs);
  }

  public void runShooter(double _________){ // Write the parameter the command needs to input
    motor1.set(_________); // Write the same parameter here
    motor2.set(_________); // Do it for the bottom motor too
  }

  public void stopShooter(){
    _________._________(); // Call the method which stops the motor from running
    _________._________(); // Call the same method for the bottom motor here
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}