package frc.robot;

public class Constants {

    public static class OperatorConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }
    public static class Extension{
        public static final int EXTENSION_MOTOR_ID = 100000;

        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final int kP = 3;
        public static final int kI = 0;
        public static final int kD = 0;
        
        public static final int CRUISE_VELOCITY = 40;
        public static final int ACCELERATION = 100;
        public static final int JERK = 1000;
        public static final double GEAR_RATIO = 21/1.0;
        public static final double FEED_FORWARD = 1;

    }

    public static class Intake{
        public static final int INTAKE_MOTOR_ID = 9;
        public static final int INTAKE_MOTOR_SPEED = 10;
    }

    public static class Indexer{
        public static final int INDEXER_MOTOR_ID = 9;

    }

    public static class Pooper{
        public static final int POOPER_TOP_MOTOR_ID = 9;
        public static final int POOPER_BOTTOM_MOTOR_ID = 10;
    }


}
