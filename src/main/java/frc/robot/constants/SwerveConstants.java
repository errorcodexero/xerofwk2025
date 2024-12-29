package frc.robot.constants;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Mass ;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.Units ;

public class SwerveConstants {
    public static final double kMaxRotationalSpeed = 1.5 * Math.PI ;
    public static final double kMaxRotationalAccel = 15 * Math.PI ;
    public static final double kRotateP = 5.0 ;
    public static final double SlowFactor = 0.1 ;

    public static final Mass kRobotMass = Units.Kilograms.of(1.0) ;
    public static final MomentOfInertia kRobotMOI = Units.KilogramSquareMeters.of(1.0) ;
    public static final double wheelCOF = 1.0 ;
    public static final Current kDriveMotorCurrentLimit = Units.Amps.of(60.0) ;
}
