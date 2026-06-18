package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public final class Constants {
    public static class OperatorConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static class Hood {
        public static final int HOOD_MOTOR_ID = 19;        
        public static final int HOOD_CANCODER_ID = 21;     

        public static final double CRUISE_VELOCITY = 500;  // TODO: test if this sped it up, originally 500
        public static final double ACCELERATION = 1000;     // TODO: test, originally 1000
        public static final double JERK = 550;//;            // placeholder

        public static final double kP = 2.5;  // placeholder 
        public static final double kI = 0.0;  // placeholder
        public static final double kD = 0.0;  // placeholder
        public static final double kS = 0.2;  // placeholder
        public static final double kV = 0.7;  // placeholder
        public static final double kA = 0.0;  // placeholder

        public static final double GEAR_RATIO = 3/1;      // motor_rot / hood_rot

        //public static final double MIN_DEG = 30.0;
        //public static final double MAX_DEG = 86.0;

        public static final double HOOD_HOLD_DEG = 30.0;
        public static final double HOOD_MAX = 480 * Constants.Hood.DEGREES_TO_REVS; //57 //30
        public static final double HOOD_MIN = 206 * Constants.Hood.DEGREES_TO_REVS; //57 //30
        public static final double HOOD_TOLERANCE_DEG = 5.00;
        public static final double HOOD_ANGLE = 280.0 * Constants.Hood.DEGREES_TO_REVS;

        public static final double DEGREES_TO_REVS = 1.0/360.0;

        public static final double OFFSET = 0.0; //-108.45703125
        public static final double PERCENT_UP = 0.5;

        public static final boolean verboseLogging = false || Logging.verboseLogging;
    }

    public static class LED {
        public static final int CANDLE_ID = 23;

        public static final boolean verboseLogging = false || Logging.verboseLogging;
    }

    public static class Yeeter {
        public static final int YEETER_MOTOR1_ID = 37; 
        public static final int YEETER_MOTOR2_ID = 18; 

        public static final double kP = 5.0; // placeholder //0.8 5ft
        public static final double kI = 0.0; // placeholder
        public static final double kD = 0.0; // placeholder
        public static final double kS = 1.3; // placeholder
        public static final double kV = 0.08; // placeholder
        public static final double kA = 0.0; // placeholder

        public static final double CRUISE_VELOCITY = 100.0; // placeholder
        public static final double ACCELERATION = 300.0;   // placeholder
        public static final double JERK = 800.0;           // placeholder

        public static final double PRELOAD_YEETER_SPEED = 700.0; 
        
        public static final double YEETER_SPEED = 65.0; //81; //rotations per second
        public static final double RESTING_SPEED = 60.0;
        public static final double YEETER_ACCELERATION = 500.0; //500
        public static final double YEETER_AUTO_SPEED = 67.5;
        public static final double YEETER_SPEED_ADDITION = 4.0;
        public static final double YEETER_FAR_SHUTTLE = 500;

        public static final double YEETER_THRESHOLD_SPEED1 = 0.5* Constants.Yeeter.YEETER_SPEED;
        //public static final double SHOOTER_THRESHOLD_SPEED2 = -86.0;

        public static final double GEAR_RATIO = 1.333; 

        public static final boolean verboseLogging = false || Logging.verboseLogging;
    }

    public static class Pivot{
        public static final int MOTOR_ID = 10;
        public static final int ENCODER_ID = 27;

        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kP = 7;
        public static final int kI = 0;
        public static final int kD = 0;

        
        public static final int CRUISE_VELOCITY_FAST = 40;
        public static final int ACCELERATION_FAST = 100;
        public static final int JERK_FAST = 1000;
        public static final double GEAR_RATIO = 20.25/1.0;
        public static final double FEED_FORWARD = 1;

        public static final int CRUISE_VELOCITY_SLOW = 5;
        public static final int ACCELERATION_SLOW = 1;
        public static final int JERK_SLOW = 10;

        public static final double PIVOT_SPEED = 10.0; //change later WHEN TESTING
        public static final double PIVOT_SLOW_SPEED = 5.0;//change later WHEN TESTING
        

        public static final double THRESHOLD = 1;
        public static final double DEGREES_TO_REVS = 1.0/360.0;
        public static final double MAGNET_OFFSET = 0.043212890625;

        public static final boolean verboseLogging = false || Logging.verboseLogging;
        public static final double DOWN_POSITION = 130 * DEGREES_TO_REVS;
        public static final double SAFE = 5 * DEGREES_TO_REVS;
        public static final double SLIGHTLY_UP_FROM_DOWN = 50 * DEGREES_TO_REVS;
        
        public static final double JITTER_POS_ONE = 100 * DEGREES_TO_REVS;
        public static final double JITTER_POS_TWO = 130 * DEGREES_TO_REVS;
        
        public static final double JITTER_POS_THREE = 120 * DEGREES_TO_REVS;
        public static final double JITTER_POS_FOUR = 90 * DEGREES_TO_REVS;

        public static final double JITTER_POS_FIVE = 100 * DEGREES_TO_REVS;
        public static final double JITTER_POS_SIX = 75 * DEGREES_TO_REVS;
    }

    public static class Eater{
        public static final int EATER_MOTOR_ID = 11;
        public static final double EATER_MOTOR_SPEED = -100.0;
        public static final boolean verboseLogging = false || Logging.verboseLogging;
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

        public static final boolean verboseLogging = false || Logging.verboseLogging;
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

        public static final boolean verboseLogging = false || Logging.verboseLogging;
    }
  
    public static class Climb {
        public static final double GEAR_RATIO = 27.5;
        public static final double JERK = 600.0;
        public static final double FEED_FORWARD = 0.0;

        public static final int MOTOR_ID = 9;
        public static final int CRUISE_VELOCITY = 30;//10, 160
        public static final int ACCELERATION = 210;//240
        public static final int CURRENT_LIMIT = 40;
        public static final int ERROR_THRESHOLD = 2;
        public static final int UP_SPEED = 10;
        public static final int DOWN_SPEED = -10;

        public static final double kP = 3;//2
        public static final int kI = 0; 
        public static final int kD = 0; 
        public static final double kS = 0.2; //0.12
        public static final int kV = 0;
        public static final int kA = 0;

        public static final double MIN_HEIGHT_METERS = 0.0;
        public static final double MAX_HEIGHT_METERS = 0.762;
        public static final double SAFE_METERS = 0.01;

        public static final double METER_CONVERSION_FACTOR = 0.0254;

        public static final double SPROCKET_PITCH_CIRCUMFERENCE = 1.432*Math.PI; //inches
        public static final double CARRIAGE_MASS_KG = 3;
        public static final double SPROCKET_RADIUS = (Constants.Climb.SPROCKET_PITCH_CIRCUMFERENCE) / (2.0 * Math.PI);
        public static final int BOTTOM_SENSOR_CHANNEL = 8;
        public static final int TOP_SENSOR_CHANNEL = 9;

        public static final boolean verboseLogging = false || Logging.verboseLogging;
    }

    public static class Visabelle {
        public static final Translation2d RED_HUB_POSE = new Translation2d(11.914324760437012, 4.033950328826904);
        public static final Translation2d BLUE_HUB_POSE = new Translation2d(4.622838497161865, 4.033950328826904);

        public static final Pose2d RED_TOWER = new Pose2d(15.421048, 3.432656, Rotation2d.kPi);
        public static final Pose2d BLUE_TOWER = new Pose2d(1.092, 4.61, Rotation2d.kZero);
        
        public static final double DIST_THRESHOLD = 3; // in meters? TODO: check and change

        public static final boolean verboseLogging = false || Logging.verboseLogging;

        public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // front ll
        public static final String kLimelightATableName = "limelight-front";
        public static final double kRobotToCameraAForward = 0.335;
        public static final double kRobotToCameraASide = -0.327;
        public static final double kCameraAHeightOffGroundMeters = 0.538;
        public static final double kCameraAPitchRads = Units.degreesToRadians(34);
        public static final Rotation2d kCameraAYawOffset = Rotation2d.fromDegrees(0.0);

        // back ll
        public static final String kLimelightBTableName = "limelight-back";
        public static final double kRobotToCameraBForward = -0.327025;
        public static final double kRobotToCameraBSide = -0.0047625;
        public static final double kCameraBHeightOffGroundMeters = 0.275717;
        public static final double kCameraBPitchRads = Units.degreesToRadians(9);
        public static final Rotation2d kCameraBYawOffset = Rotation2d.fromDegrees(180.0);

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