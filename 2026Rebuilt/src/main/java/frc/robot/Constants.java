// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {
    public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    }

public static class Pivot {
    public static final int PIVOT_MOTOR_ID = 100000;

    public static final int kS = 0;
    public static final int kV = 0;
    public static final int kA = 0;
    public static final int kP = 0;
    public static final int kI = 0;
    public static final int kD = 0;
    
    public static final int CRUISE_VELOCITY = 0;
    public static final int ACCELERATION = 0;
    public static final int JERK = 0;
    public static final int GEAR_RATIO = 1;
}
 public static class Climb {
    public static final int RIGHT_MOTOR_ID = 90;
    public static final int LEFT_MOTOR_ID = 99;
    public static final int WIND_FACTOR = 3;
    public static final int UNWIND_FACTOR = 4;
    public static final int CRUISE_VELOCITY = 10;
    public static final int ACCELERATION = 20;
    public static final int CURRENT_LIMIT = 40;

    public static final int UP_kP = 1;
    public static final int UP_kI = 2;
    public static final int UP_kD = 3;
    public static final int UP_kS = 4;

    
    public static final int DOWN_kP = 1;
    public static final int DOWN_kI = 2;
    public static final int DOWN_kD = 3;
    public static final int DOWN_kS = 4;
  
}
}