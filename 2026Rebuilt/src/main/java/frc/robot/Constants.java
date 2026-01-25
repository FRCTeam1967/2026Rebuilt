// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Hood {
     public static final int HOOD_MOTOR_ID = 17; //change number later

     public static final double CRUISE_VELOCITY = 50.0; //placeholder
     public static final double ACCELERATION = 80.0; //placeholder
     public static final double JERK = 110.0; //placeholder

    public static final double kP = 0.5; //placeholder
    public static final double kI = 0.0; //placeholder
    public static final double kD = 0.0; //placeholder
    public static final double kS = 0.0; //placeholder
    public static final double kV = 0.0; //placeholder
    public static final double kA = 0.0; //placeholder

    public static final double CURRENT_LIMIT = 0.0; //placeholder
    public static final double GEAR_RATIO = 0.0; //placeholder

    public static final double HOOD = 0.0; //placeholder
  }

  public static class FlywheelShooter {
     public static final int FLYWHEELSHOOTER_MOTOR1_ID = 16; //change number later
     public static final int FLYWHEELSHOOTER_MOTOR2_ID = 17; //change number later

    public static final double kP = 0.5; //placeholder
    public static final double kI = 0.0; //placeholder
    public static final double kD = 0.0; //placeholder
    public static final double kS = 0.0; //placeholder
    public static final double kV = 0.0; //placeholder
    public static final double kA = 0.0; //placeholder

    public static final double CRUISE_VELOCITY = 0.0; //placeholder
    public static final double ACCELERATION = 0.0; //placeholder
    public static final double JERK = 0.0; //placeholder
    
    public static final double GEAR_RATIO = 5.0; 

    public static final double FLYWHEEL_SHOOTER = 0.0;
  }

  public static class Xbox {
     public static final int OPERATOR_CONTROLLER_PORT = 0; //change number later
  }
}
