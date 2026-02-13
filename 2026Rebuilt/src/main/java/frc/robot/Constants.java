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
    public static final int HOOD_MOTOR_ID = 19;        
    public static final int HOOD_CANCODER_ID = 21;     

    public static final double CRUISE_VELOCITY = 1.0;  // placeholder
    public static final double ACCELERATION = 2.0;     // placeholder
    public static final double JERK = 10.0;            // placeholder

    public static final double kP = 0.5;  // placeholder
    public static final double kI = 0.0;  // placeholder
    public static final double kD = 0.0;  // placeholder
    public static final double kS = 0.0;  // placeholder
    public static final double kV = 0.0;  // placeholder
    public static final double kA = 0.0;  // placeholder

    public static final double GEAR_RATIO = 11.4;      // motor_rot / hood_rot

    public static final double MIN_DEG = 30.0;
    public static final double MAX_DEG = 86.0;

    public static final double HOOD_HOLD_DEG = 30.0;
    public static final double HOOD_TEST_SHOT = 35.0; //use this for testing
    public static final double HOOD_TOLERANCE_DEG = 2.0;

  }

  public static class FlywheelShooter {
    public static final int FLYWHEELSHOOTER_MOTOR1_ID = 37; 
    public static final int FLYWHEELSHOOTER_MOTOR2_ID = 18; 

    public static final double kP = 0.5; // placeholder
    public static final double kI = 0.0; // placeholder
    public static final double kD = 0.0; // placeholder
    public static final double kS = 0.0; // placeholder
    public static final double kV = 0.0; // placeholder
    public static final double kA = 0.0; // placeholder

    public static final double CRUISE_VELOCITY = 50.0; // placeholder
    public static final double ACCELERATION = 200.0;   // placeholder
    public static final double JERK = 800.0;           // placeholder

    public static final double FLYWHEEL_SHOOTER_SPEED1 = 40.0; //rotations per second
    public static final double FLYWHEEL_SHOOTER_SPEED2 = 50.0;
    public static final double FLYWHEEL_SHOOTER_SPEED3 = 60.0;

    public static final double GEAR_RATIO = 1.3333; 
  }

  public static class Xbox {
    public static final int OPERATOR_CONTROLLER_PORT = 0; // change later
  }
}
