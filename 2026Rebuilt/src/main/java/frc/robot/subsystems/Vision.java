// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;  
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private double maxAngularRate;

  public Vision(double maxAngularRate) {
    this.maxAngularRate = maxAngularRate;
  }

  public double limelight_aim_proportional() {        
      double kP = 0.02; //0.035
      double targetingAngularVelocity = 0.0; 
      // tx ranges from (-hfov/2) to (hfov/2) in degrees.

      targetingAngularVelocity = (LimelightHelpers.getTX("limelight-front") * kP);

      // convert to radians per second for our drive method
      targetingAngularVelocity *= maxAngularRate;
      //invert since tx is positive when the target is to the right of the crosshair
      targetingAngularVelocity *= -1.0;
      return targetingAngularVelocity;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
