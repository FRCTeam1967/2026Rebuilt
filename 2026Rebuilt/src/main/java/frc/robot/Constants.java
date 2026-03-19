package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public final class Constants {
    public static class OperatorConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static class Hood {
        public static final int HOOD_MOTOR_ID = 19;        
        public static final int HOOD_CANCODER_ID = 14;     

        public static final double CRUISE_VELOCITY = 500;  // placeholder
        public static final double ACCELERATION = 300;     // placeholder
        public static final double JERK = 300;            // placeholder

        public static final double kP = 5.0;  // placeholder
        public static final double kI = 0.0;  // placeholder
        public static final double kD = 0.0;  // placeholder
        public static final double kS = 0.5;  // placeholder
        public static final double kV = 0.3;  // placeholder
        public static final double kA = 0.0;  // placeholder

        public static final double GEAR_RATIO = 3/1;      // motor_rot / hood_rot

        //public static final double MIN_DEG = 30.0;
        //public static final double MAX_DEG = 86.0;

        public static final double HOOD_HOLD_DEG = 30.0;
        public static final double HOOD_MAX = 680.0 * Constants.Hood.DEGREES_TO_REVS; //57 //30
        public static final double HOOD_MIN = 330.0 * Constants.Hood.DEGREES_TO_REVS; //57 //30
        public static final double HOOD_TOLERANCE_DEG = 5.00;
        public static final double HOOD_ANGLE = 390.0 * Constants.Hood.DEGREES_TO_REVS;

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
        public static final int YEETER_MOTOR1_ID = 30; 
        public static final int YEETER_MOTOR2_ID = 28; 

        public static final double kP = 0.7; // placeholder //0.8 5ft
        public static final double kI = 0.0; // placeholder
        public static final double kD = 0.0; // placeholder
        public static final double kS = 0.0; // placeholder
        public static final double kV = 0.13; // placeholder
        public static final double kA = 0.0; // placeholder

        public static final double CRUISE_VELOCITY = 100.0; // placeholder
        public static final double ACCELERATION = 300.0;   // placeholder
        public static final double JERK = 800.0;           // placeholder

        public static final double PRELOAD_YEETER_SPEED = 700.0; 
        
        public static final double YEETER_SPEED = 67.5; //81; //rotations per second
        public static final double YEETER_ACCELERATION = 500.0; //500

        public static final double YEETER_THRESHOLD_SPEED1 = 0.5* Constants.Yeeter.YEETER_SPEED;
        //public static final double SHOOTER_THRESHOLD_SPEED2 = -86.0;

        public static final double GEAR_RATIO = 1.333; 

        public static final boolean verboseLogging = false || Logging.verboseLogging;
    }

    public static class Pivot{
        public static final int MOTOR_ID = 11;
        public static final int ENCODER_ID = 25;

        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kP = 5;
        public static final int kI = 0;
        public static final int kD = 0;
        
        public static final int CRUISE_VELOCITY = 40;
        public static final int ACCELERATION = 100;
        public static final int JERK = 1000;
        public static final double GEAR_RATIO = 20.25/1.0;
        public static final double FEED_FORWARD = 1;
        

        public static final double THRESHOLD = 1;
        public static final double DEGREES_TO_REVS = 1.0/360.0;
        public static final double MAGNET_OFFSET = -0.456787109375;

        public static final double DOWN_POSITION = 165 * DEGREES_TO_REVS;
        public static final double SLIGHTLY_UP_FROM_DOWN = 65 * DEGREES_TO_REVS;
        public static final double SAFE = 40 * DEGREES_TO_REVS;

        public static final boolean verboseLogging = false || Logging.verboseLogging;
    }

    public static class Eater{
        public static final int EATER_MOTOR_ID = 9;
        public static final double EATER_MOTOR_SPEED = -120.0;

        public static final boolean verboseLogging = false || Logging.verboseLogging;
    }

    public static class Indexer{
        public static final int INDEXER_MOTOR_ID = 12;
        public static final int INDEXER_SPEED = 10;

        public static final boolean verboseLogging = false || Logging.verboseLogging;
    }

    public static class Feeder{
        public static final int FEEDER_MOTOR_ID = 13;
        public static final double FEEDER_SPEED = -40.0; //-20 //-10

        public static final double PREP_FEEDER = 50.0; 

        public static final boolean verboseLogging = false || Logging.verboseLogging;
    }
  
    public static class Climb {
        public static final double GEAR_RATIO = 27.5;
        public static final double JERK = 600.0;
        public static final double FEED_FORWARD = 0.0;

        public static final int MOTOR_ID = 10;
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
        public static final double DEGREE_ERROR = 4.0;
        public static final double LIMELIGHT_ANGLE_DEGREES = 0; //need to verify
        public static final double LIMELIGHT_HEIGHT_INCHES = 25.125; //need to verify
        public static final double TARGET_HEIGHT_INCHES = 54; //need to verify
        public static final double LIMELIGHT_ALIGN_LEFT_OFFSET = -0.156;//-0.125;//11.0812301635 //10.6631250381 //10.234612469perhaps in //-0.1175; // -0.125 //-0.1175; //-0.1256; //-0.1556 /-0.181 //-0.18129 //-0.10509 //-0.0889-0.0762 // -0.1200 //-0.14605
        public static final double LIMELIGHT_ALIGN_RIGHT_OFFSET = 0.2;//0.18; //-11.5789194107 //-11.9671554656/0.15875; //0.17145 //0.1524 //0.1905; //0.18415; //0.1685 //0.1513 //0.2275-0.076 //0.2286, left, -.0658890 // 0.1885 // 0.1905
        public static final double LIMELIGHT_ALIGN_Z_OFFSET = 0.0; //LimelightHelpers.setFiducial3DOffset("limelight", 0.0, 0.0, 0.001);
        public static final double LIMELIGHT_ALIGN_CENTER_OFFSET = -0.1431036;
        public static final double LIMELIGHT_AUTO_LEFT_OFFSET = -0.11;
        public static final double LIMELIGHT_L4_LEFT_OFFSET = -0.1;
        public static final double ALIGNMENT_SPEED = (Math.abs(Math.pow(0.5, 3))> 0 ? Math.pow(0.5, 3) : 0) * TunerConstants.maxSpeed; //0.43 //0.425
        public static final double ALIGNMENT_THRESHOLD = 1.0;
        public static final double FORWARD_ALIGNMENT_THRESHOLD = 0.0;
        public static final double ALIGNMENT_LEFT_OFFSET = 9.2; //10.54;//10.74;//1.0; //1.1; //9.76; //TODO: test at EPA
        public static final double ALIGNMENT_RIGHT_OFFSET = -9.2;//-11.86;//-0.8; //-1.0; //-1.1; //-13.1; //in LL degrees //TODO: test at EPA
        public static final double ALIGNMENT_FORWARD_OFFSET = 0.0; //TODO: test at EPA //-5.16 center for left branch aligned

        public static final double ALIGNMENT_X_KP = -0.17;  //0.708 // Used for aligning Robot X (forward), which is "ty" in Limelight terms
        public static final double ALIGNMENT_Y_KP = -0.03;  //-0.05 // Used for aligning Robot Y (side-side), which is "tx" in Limelight terms

        public static final Translation2d RED_HUB_POSE = new Translation2d(11.914324760437012, 4.033950328826904);
        public static final Translation2d BLUE_HUB_POSE = new Translation2d(4.622838497161865, 4.033950328826904);

        public static final boolean verboseLogging = false || Logging.verboseLogging;
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
        public static boolean verboseLogging = true;

        /**
         * Enabling this will start the CTRE signal logger.
         */
        public static boolean enableCTRELogging = true;

        public static boolean enableExtras = true;      // Enable "extras"
        public static boolean capturePDH = true;        // Requires "extras"
        public static boolean captureDS = true;         // Capture driver station input (joystick, etc.)
        public static boolean captureNT = false;        // Capture network table values -- probably expensive, but some teams do this
        public static boolean captureConsole = true;    // Capture console output like loop overruns
    }
}