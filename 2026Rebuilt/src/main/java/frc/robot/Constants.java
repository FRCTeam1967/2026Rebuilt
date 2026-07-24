package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.TunerConstants;

public final class Constants {
    public static class OperatorConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    // public static class LED {
    //     public static final int CANDLE_ID = 23;

    //     public static final boolean verboseLogging = false || Logging.verboseLogging;
    // }

    public static class IntakeShooter {
        public static final int INTAKE_SHOOTER_MOTOR_ID = 37; 
        //public static final int SHOOTER_MOTOR2_ID = 18; 

        public static final double kP = 5.0; // placeholder //0.8 5ft
        public static final double kI = 0.0; // placeholder
        public static final double kD = 0.0; // placeholder
        public static final double kS = 1.3; // placeholder
        public static final double kV = 0.08; // placeholder
        public static final double kA = 0.0; // placeholder

        public static final double CRUISE_VELOCITY = 100.0; // placeholder
        public static final double ACCELERATION = 300.0;   // placeholder
        public static final double JERK = 800.0;           // placeholder

        public static final double PRELOAD_SHOOTER_SPEED = 700.0; 
        
        public static final double SHOOTER_SPEED = 65.0; //81; //rotations per second
        public static final double RESTING_SPEED = 60.0;
        public static final double SHOOTER_ACCELERATION = 500.0; //500
        public static final double SHOOTER_AUTO_SPEED = 67.5;
        public static final double SHOOTER_SPEED_ADDITION = 4.0;
        public static final double SHOOTER_FAR_SHUTTLE = 500;

        public static final double SHOOTER_THRESHOLD_SPEED = 0.5* Constants.IntakeShooter.SHOOTER_SPEED;

        public static final double GEAR_RATIO = 1.333; 
        public static final double INTAKE_MOTOR_SPEED = -100.0;
        public static final double SHOOTER_SPEED_HUB = 67.5;
        public static final double SHOOTER_SPEED_CENTER = 67.5;
        public static final double SHOOTER_SPEED_BEHIND_LB = 67.5;
        public static final double SHOOTER_SPEED_BEHIND_RB = 67.5;
        public static final double SHOOTER_SPEED_FRONT_TW = 67.5;
        public static final double SHOOTER_SPEED_FRONT_D = 67.5;
        public static final double SHOOTER_SPEED_FRONT_O = 67.5;
        public static final double SHOOTER_SPEED_BEHIND_DT = 67.5;
        public static final double SHOOTER_SPEED_BEHIND_OT = 67.5;
        public static final double SHOOTER_SPEED_INSIDE_TW = 67.5;
        public static final double SHOOTER_SPEED_LEFT_TW = 67.5;
        public static final double SHOOTER_SPEED_RIGHT_TW = 67.5;
        public static final double SHOOTER_SPEED_LEFT_D = 67.5;
        public static final double SHOOTER_SPEED_LEFT_O = 67.5;
        public static final double SHOOTER_SPEED_O_SHOOT = 67.5;
        public static final double SHOOTER_SPEED_D_SHOOT = 67.5;
        public static final double SHOOTER_THRESHOLD_ERROR = 5;

        //public static final boolean verboseLogging = false || Logging.verboseLogging;
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

        //public static final boolean verboseLogging = false || Logging.verboseLogging;
    }
  
    public static class Drivetrain {
        public static final boolean verboseLogging = true;
    }

    public static class Logging {
        /**
         * Enabling this will enable DogLog.log(). Setting it to false will cause all logging to be disabled.
         */
        public static boolean enabled = true;

        /**
         * Enabling this will enable verbose logging across all subsystems
         */
        public static boolean verboseLogging = false;

        /**
         * Enabling this will start the CTRE signal logger.
         */
        public static boolean enableCTRELogging = true;

        /**
         * Control aspects of DogLog logging
         */
        public static boolean enableExtras = true;      // Enable "extras"
        public static boolean capturePDH = false;        // Requires "extras"
        public static boolean captureDS = true;         // Capture driver station input (joystick, etc.)
        public static boolean captureNT = false;        // Capture network table values -- probably expensive, but some teams do this
        public static boolean captureConsole = true;    // Capture console output like loop overruns

        /**
         * Control aspects of CTRE logging.
         */
        public static class CTRE {
            /**
             * Is NetworkTable publishing enabled. CTRE default is true, but this could be expensive. They log via 
             * SignalLogger too, and we log some of the same state via DogLog, so we should disable this if we're
             * still seeing NT publishing cause loop overruns when we've disabled our NT logging.
             */
            public static boolean enableNTPublishing = false; 
    
        }
    }
}
