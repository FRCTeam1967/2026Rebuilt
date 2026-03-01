package frc.robot;

public class Constants {

    public static class OperatorConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
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
       public static final double GEAR_RATIO = 36/1.0;
       public static final double FEED_FORWARD = 1;
    
       public static final double THRESHOLD = 1;
       public static final double DEGREES_TO_REVS = 1.0/360.0;
       public static final double MAGNET_OFFSET = -0.416259765625;


       public static final double DOWN_POSITION = 120 * DEGREES_TO_REVS;
       public static final double SAFE = 5 * DEGREES_TO_REVS;
   }


    public static class Intake{
       public static final int INTAKE_MOTOR_ID = 9;
       public static final int INTAKE_MOTOR_SPEED = 30;
   }

   public static class Indexer{
       public static final int INDEXER_MOTOR_ID = 12;
       public static final int INDEXER_SPEED = 10;

   }



}
