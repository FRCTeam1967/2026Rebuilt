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
    public static final int FLYWHEELSHOOTER_MOTOR1_ID = 18; 
    public static final int FLYWHEELSHOOTER_MOTOR2_ID = 37; 

    public static final double kP = 0.67; // placeholder
    public static final double kI = 0.0; // placeholder
    public static final double kD = 0.0; // placeholder
    public static final double kS = 0.0; // placeholder
    public static final double kV = 0.13; // placeholder
    public static final double kA = 0.0; // placeholder

    public static final double CRUISE_VELOCITY = 100.0; // placeholder
    public static final double ACCELERATION = 300.0;   // placeholder
    //public static final double JERK = 800.0;           // placeholder

    public static final double PRELOAD_SHOOTER_SPEED = 70.0; 
    public static final double FLYWHEEL_SHOOTER_SPEED = 2.0; //75; //rotations per second
    public static final double FLYWHEEL_SHOOTER_ACCELERATION = 200.0;

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
        public static final double kP = 1.5;
        public static final int kI = 0;
        public static final int kD = 0;
        
        public static final int CRUISE_VELOCITY = 40;
        public static final int ACCELERATION = 100;
        public static final int JERK = 1000;
        public static final double GEAR_RATIO = 36/1.0;
        public static final double FEED_FORWARD = 1;
        public static final int SAFE = 66;

        public static final double THRESHOLD = 1;
        public static final double DEGREES_TO_REVS = 0.0277778;
        public static final double MAGNET_OFFSET = -120.849609375;

        public static final double DOWN_POSITION = 290; // * DEGREES_TO_REVS;
    }

    public static class Intake{
        public static final int INTAKE_MOTOR_ID = 11;
        public static final int INTAKE_MOTOR_SPEED = 15;
    }

    public static class Indexer{
        public static final int INDEXER_MOTOR_ID = 12;
        public static final int INDEXER_SPEED = 10;

    }

    public static class Feeder{
        public static final int FEEDER_MOTOR_ID = 36; //TODO: change this to actual ID
        public static final double FEEDER_SPEED = -10.0;
    }


}
