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
    public static class Pivot{
       public static final int MOTOR_ID = 11;
       public static final int ENCODER_ID = 0;


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
       public static final double MAGNET_OFFSET =   0.460693359375;//0.419921875


       public static final double DOWN_POSITION = 135 * DEGREES_TO_REVS;
       public static final double SAFE = 5 * DEGREES_TO_REVS;
   }


    public static class Intake{
       public static final int INTAKE_MOTOR_ID = 9;
       public static final int INTAKE_MOTOR_SPEED = 70;
   }

   public static class Indexer{
       public static final int INDEXER_MOTOR_ID = 12;
       public static final int INDEXER_SPEED = 10;

   }



}
