// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    }

 public static class Climb {

    public static final double GEAR_RATIO = 64.0/8.0;
    public static final double JERK = 600.0; //1000 //1600 <---how fast the acceleration is reached
    public static final double FEED_FORWARD = 0.0;

    public static final int MOTOR_ID = 90;
    public static final int CRUISE_VELOCITY = 160;
    public static final int ACCELERATION = 240;
    public static final int CURRENT_LIMIT = 40;
    public static final int ERROR_THRESHOLD = 2;

    public static final double kP = 0.5;
    public static final int kI = 0; 
    public static final int kD = 0; 
    public static final int kS = 0;  
    public static final int kV = 0;
    public static final int kA = 0;

    public static final double MIN_HEIGHT_METERS = 0.0;
    public static final double MAX_HEIGHT_METERS = 0.762;
    public static final double SAFE_METERS = 0.01;

    public static final double METER_CONVERSION_FACTOR = 0.0254;

    public static final double SPROCKET_PITCH_CIRCUMFERENCE = 1.76*METER_CONVERSION_FACTOR*Math.PI;
    public static final double CARRIAGE_MASS_KG = 3;
    public static final double SPROCKET_RADIUS = (Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE) / (2.0 * Math.PI);
    public static final int SENSOR_CHANNEL = 0;
}
public static class TelescopingArm {
    public static final int RIGHT_MOTOR_ID = 1000;
    public static final int DIGITAL_INPUT_ID = 10000;
    public static final double CURRENT_LIMIT = 0;
    public static final double DEADBAND = 0.05;
    public static final double UNWIND_FACTOR = 1;
    public static final double WIND_FACTOR = 0.5;
    public static final double TOP_ROTATIONS = 0;
    public static final double LOWER_SPEED = 0;
}
  public static class Swerve {
    //motor/encoder ids
    public static final int FL_POWER = 7, FL_STEER = 8, FL_ENCODER = 4;
    public static final int FR_POWER = 1, FR_STEER = 2, FR_ENCODER = 1;
    public static final int BL_POWER = 5, BL_STEER = 6, BL_ENCODER = 3;
    public static final int BR_POWER = 3, BR_STEER = 4, BR_ENCODER = 2;

    public static final double FL_OFFSET = -135.439453125/360; //-136.40625/360;
    //public static final double FR_OFFSET = 1.0546875/360;//-0.3515625/360;
    public static final double FR_OFFSET = (177.099609375+180)/360;//-0.3515625/360;
    public static final double BL_OFFSET = 117.158203125/360; //115.048828125;
    public static final double BR_OFFSET = -14.150390625/360; //-17.05078125/360;
    public static final int PIGEON_GYRO = 9;

    //pid values
    public static final double POWER_kS = 0.14;//0.14; 
    public static final double POWER_kV = 12/(100/8.14); //1.25; 
    public static final double POWER_kA = 0; 
    public static final double POWER_kP = POWER_kV * 0.8; //0.25;
    public static final double POWER_kI = 0;
    public static final double POWER_kD = 0;

    public static final double STEER_kS = 0; //0.1
    public static final double STEER_kV = 0; //30
    public static final double STEER_kA = 0; // 15
    public static final double STEER_kP = 12; //100
    public static final double STEER_kI = 0;
    public static final double STEER_kD = 0.5;

    //gear ratios + meter conversions
    public static final double STEER_GEAR_RATIO = 150.0/7.0;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;
    public static final double MK4I_L1_REV_TO_METERS = WHEEL_CIRCUMFERENCE;
    public static final double RPM_TO_MPS = MK4I_L1_REV_TO_METERS / 60.0;
    public static final double SENSOR_ROTATION_TO_MOTOR_RATIO = STEER_GEAR_RATIO;
    public static final double REVERSE_OFFSET = Units.inchesToMeters(2.0) * Math.PI;
    public static final double METERS_TO_ENC_COUNT = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    

    //distances/measurements
    public static final double SWERVE_MAX_SPEED = 4.1695; //m/s
    public static final double WIDTH = Units.inchesToMeters(23), LENGTH = Units.inchesToMeters(23);
    public static final double SWERVE_AMP_OFFSET = 0.3083496; //rotations of encoder
    public static final double AMP_REVERSE_JS_INPUT = 0.4; //joystick input

    //max speeds
    public static final double ROTATION_CIRCLE_CIRCUMFERENCE = (WIDTH / Math.sqrt(2.0)) * 2.0 * Math.PI;
    public static final double SWERVE_ROTATION_MAX_SPEED_IN_RAD = (SWERVE_MAX_SPEED / ROTATION_CIRCLE_CIRCUMFERENCE) * 2.0 * Math.PI; 

    //kinematics
    public static Translation2d m_frontLeftLocation = new Translation2d(LENGTH / 2, WIDTH / 2);
    public static Translation2d m_frontRightLocation = new Translation2d(LENGTH / 2, -WIDTH / 2);
    public static Translation2d m_backLeftLocation = new Translation2d(-LENGTH / 2, WIDTH / 2);
    public static Translation2d m_backRightLocation = new Translation2d(-LENGTH / 2, -WIDTH / 2);

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );
    
    public static final double SWERVE_ROTATION_TOLERANCE = 5;

    public static final TrapezoidProfile.Constraints SWERVE_ROTATION_PID_CONSTRAINTS = new TrapezoidProfile.Constraints(36000, 36000);
    public static final TrapezoidProfile.Constraints SWERVE_TRANSLATION_PID_CONSTRAINTS = new TrapezoidProfile.Constraints(15, 3);
  }
  public static class ExperimentalFeatures {
    public static final boolean useCosineCompensation = true; 
    public static final boolean disableRotationWhenNotMoving = false;
    public static final boolean applyDriverDeadband = false;

    // Setting this to true because that's how the code works, but there's no reason to do this AFAICT, and
    // it causes unnecessary CAN traffic. Each call to the 4 modules is BLOCKING. CTRE says:
    // "We recommend that users avoid calling this API periodically"
    // So this should really be set to FALSE.
    public static final boolean applyNeutralModeConstantlyInAuto = false;
  }
}