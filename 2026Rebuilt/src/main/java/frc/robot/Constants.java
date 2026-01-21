// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {
    public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    }

 public static class Climb {

    public static final double GEAR_RATIO = 64/8;
    public static final double JERK = 600; //1000 //1600 <---how fast the acceleration is reached
    public static final double FEED_FORWARD = 0;

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

    public static final double MIN_HEIGHT = 0;
    public static final double MAX_HEIGHT = 30;
    public static final double SAFE = 0.1;

    public static final double SPROCKET_PITCH_CIRCUMFERENCE = 1.76*Math.PI;
    public static final double CARRIAGE_MASS_KG = 3;
    public static final double SPROCKET_RADIUS = (Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE * 0.0254) / (2.0 * Math.PI);
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
}