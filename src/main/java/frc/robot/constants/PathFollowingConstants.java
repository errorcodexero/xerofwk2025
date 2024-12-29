package frc.robot.constants;

public class PathFollowingConstants {
    public static final double kXYTolerance = 0.05 ;
    public static final double kAngleTolerance = 2.0 ;
    public static final double kMaxVelocity = 60.0 ;
    public static final double kMaxAccel = 60.0 ;

    public static final class XYCtrl {
        public static final double kP = 6.0 ;
        public static final double kI = 0.0 ;
        public static final double kD = 0.0 ;
    }
    
    public static final class RotCtrl {
        public static final double kP = 8.0;
        public static final double kI = 0.0 ;
        public static final double kD = 0.0 ;
    }        
} ;

