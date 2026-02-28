package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static class OperatorConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

  public static class Hood {
    public static final int HOOD_MOTOR_ID = 19;        
    public static final int HOOD_CANCODER_ID = 21;     

    public static final double CRUISE_VELOCITY = 500;  // placeholder
    public static final double ACCELERATION = 300;     // placeholder
    public static final double JERK = 300;            // placeholder

    public static final double kP = 5.0;  // placeholder
    public static final double kI = 0.0;  // placeholder
    public static final double kD = 0.0;  // placeholder
    public static final double kS = 0.0;  // placeholder
    public static final double kV = 0.0;  // placeholder
    public static final double kA = 0.0;  // placeholder

    public static final double GEAR_RATIO = 1.5;      // motor_rot / hood_rot

    //public static final double MIN_DEG = 30.0;
    //public static final double MAX_DEG = 86.0;

    public static final double HOOD_HOLD_DEG = 30.0;
    public static final double HOOD_MAX = 57 * Constants.Hood.DEGREES_TO_REVS; //57 //30
    public static final double HOOD_TOLERANCE_DEG = 5.00;

    public static final double DEGREES_TO_REVS = 1.0/360.0;

    public static final double OFFSET = 0.0; //-108.45703125
    public static final double PERCENT_UP = 0.5;

  }

  public static class FlywheelShooter {
    public static final int FLYWHEELSHOOTER_MOTOR1_ID = 37; 
    public static final int FLYWHEELSHOOTER_MOTOR2_ID = 18; 

    public static final double kP = 0.8; // placeholder
    public static final double kI = 0.0; // placeholder
    public static final double kD = 0.0; // placeholder
    public static final double kS = 0.0; // placeholder
    public static final double kV = 0.0; // placeholder
    public static final double kA = 0.0; // placeholder

    public static final double CRUISE_VELOCITY = 100.0; // placeholder
    public static final double ACCELERATION = 300.0;   // placeholder
    //public static final double JERK = 800.0;           // placeholder

    public static final double PRELOAD_SHOOTER_SPEED = 70.0; 
    public static final double FLYWHEEL_SHOOTER_SPEED = 80.0; //75; //rotations per second
    public static final double FLYWHEEL_SHOOTER_ACCELERATION = 500.0;

    public static final double SHOOTER_THRESHOLD_SPEED1 = 0.5*Constants.FlywheelShooter.FLYWHEEL_SHOOTER_SPEED;
    //public static final double SHOOTER_THRESHOLD_SPEED2 = -86.0;

    public static final double GEAR_RATIO = 1.333; 
  }

    public static class Pivot{
        public static final int MOTOR_ID = 10;
        public static final int ENCODER_ID = 27;

        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kP = 5;
        public static final int kI = 0;
        public static final int kD = 0;
        
        public static final int CRUISE_VELOCITY = 40;
        public static final int ACCELERATION = 100;
        public static final int JERK = 1000;
        public static final double GEAR_RATIO = 36/1.0;
        public static final double FEED_FORWARD = 1;
        

        public static final double THRESHOLD = 1;
        public static final double DEGREES_TO_REVS = 1.0/360.0;
        public static final double MAGNET_OFFSET = -0.3349609375;

        public static final double DOWN_POSITION = 120 * DEGREES_TO_REVS;
        public static final double SAFE = 5 * DEGREES_TO_REVS;
    }

    public static class Intake{
        public static final int INTAKE_MOTOR_ID = 11;
        public static final int INTAKE_MOTOR_SPEED = 30;
    }

    public static class Indexer{
        public static final int INDEXER_MOTOR_ID = 12;
        public static final int INDEXER_SPEED = 10;

    }

    public static class Feeder{
        public static final int FEEDER_MOTOR_ID = 36; //TODO: change this to actual ID
        public static final double FEEDER_SPEED = -10.0;
    }

    public static class Vision {
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
    public static final double ALIGNMENT_SPEED = (Math.abs(Math.pow(0.5, 3))> 0 ? Math.pow(0.5, 3) : 0) * Swerve.SWERVE_MAX_SPEED; //0.43 //0.425
    public static final double ALIGNMENT_THRESHOLD = 1.0;
    public static final double FORWARD_ALIGNMENT_THRESHOLD = 0.0;
    public static final double ALIGNMENT_LEFT_OFFSET = 9.2; //10.54;//10.74;//1.0; //1.1; //9.76; //TODO: test at EPA
    public static final double ALIGNMENT_RIGHT_OFFSET = -9.2;//-11.86;//-0.8; //-1.0; //-1.1; //-13.1; //in LL degrees //TODO: test at EPA
    public static final double ALIGNMENT_FORWARD_OFFSET = 0.0; //TODO: test at EPA //-5.16 center for left branch aligned

    public static final double ALIGNMENT_X_KP = -0.17;  //0.708 // Used for aligning Robot X (forward), which is "ty" in Limelight terms
    public static final double ALIGNMENT_Y_KP = -0.03;  //-0.05 // Used for aligning Robot Y (side-side), which is "tx" in Limelight terms
  }


}
