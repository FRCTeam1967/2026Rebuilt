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
   }

}