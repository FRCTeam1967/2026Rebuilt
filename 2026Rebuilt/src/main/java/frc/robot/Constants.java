// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {
    public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    }

 public static class Climb {

    public static final double GEAR_RATIO = 64/8;
    public static final double JERK = 600; //1000 //1600 <---how fast the acceleration is reached
    public static final double FEED_FORWARD = 0;

    public static final int RIGHT_MOTOR_ID = 90;
    public static final int LEFT_MOTOR_ID = 99;
    public static final int WIND_FACTOR = 3;
    public static final int UNWIND_FACTOR = 4;
    public static final int CRUISE_VELOCITY = 10;
    public static final int ACCELERATION = 20;
    public static final int CURRENT_LIMIT = 40;
    public static final int ERROR_THRESHOLD = 12;

    public static final int UP_kP = 1;
    public static final int UP_kI = 2;
    public static final int UP_kD = 3;
    public static final int UP_kS = 4;

    
    public static final int DOWN_kP = 1;
    public static final int DOWN_kI = 2;
    public static final int DOWN_kD = 3;
    public static final int DOWN_kS = 4;

    public static final double SPROCKET_PITCH_CIRCUMFERENCE = 100000;
  
}
}