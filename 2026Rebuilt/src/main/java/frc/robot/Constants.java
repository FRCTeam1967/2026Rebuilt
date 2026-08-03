// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class Shooter{
        public static final int TOP_SHOOTER_ID = ________; // Set this value to 1
        public static final int BOTTOM_SHOOTER_ID = 2;

        public static final double SHOOTER_SPEED = 0.5;
    }

    public static class Indexer{
        public static final int INDEXER_MOTOR_ID = 12;
        public static final double INDEXER_SPEED = 100;

        public static final double kP = 1.5; // placeholder //0.8 5ft
        public static final double kI = 0.0; // placeholder
        public static final double kD = 0.0; // placeholder
        public static final double kS = 0.12; // placeholder
        public static final double kV = 0.0; // placeholder
        public static final double kA = 0.0; // placeholder

        public static final double CRUISE_VELOCITY = 50.0; // placeholder
        public static final double ACCELERATION = 100.0;   // placeholder
        public static final double JERK = 0.0;           // placeholder
    }

    public static class Feeder{
        public static final int FEEDER_MOTOR_ID = 36;
        public static final double FEEDER_SPEED = 100.0; //-20 //-10

        public static final double FEEDER_GEAR_RATIO = 5/1;

        public static final double PREP_FEEDER = -50.0; 

        public static final double INTAKE_FEEDER = -20.0; 

        public static final double kP = 5; // placeholder //0.8 5ft
        public static final double kI = 0.0; // placeholder
        public static final double kD = 0.0; // placeholder
        public static final double kS = 0.12; // placeholder
        public static final double kV = 0.0; // placeholder
        public static final double kA = 0.0; // placeholder

        public static final double CRUISE_VELOCITY = 50.0; // placeholder
        public static final double ACCELERATION = 100.0;   // placeholder
        public static final double JERK = 0.0;           // placeholder
    }
}