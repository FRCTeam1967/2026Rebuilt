// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pooper extends SubsystemBase {
  private TalonFX topMotor;
  private TalonFX bottomMotor;
  /** Creates a new Pooper. */

  public Pooper() {
    topMotor = new TalonFX(Constants.Pooper.POOPER_TOP_MOTOR_ID); 
    bottomMotor = new TalonFX(Constants.Pooper.POOPER_BOTTOM_MOTOR_ID);
  }

  public void setMotors(double speed){
    topMotor.set(speed);
    bottomMotor.set(speed*-1);
  }

  public void stopMotors(){
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }
    
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

