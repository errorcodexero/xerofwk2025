package frc.robot.subsystems;

import java.util.Map;

import org.xero1425.base.LimelightHelpers;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroSubsystem;
import org.xero1425.base.LimelightHelpers.PoseEstimate;
import org.xero1425.misc.SettingsValue;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.commons.PolynomialRegression;

public class VisionSubsystem extends XeroSubsystem {

    public class VisionEstimate {
        public final double xyStds ;
        public final double thetaStds ;
        public final double timeStamp ;
        public final Pose2d pose ;

        public VisionEstimate(Pose2d p, double ts, double xy, double theta) {
            pose = p ;
            timeStamp = ts ;
            xyStds = xy ;
            thetaStds = theta ;
        }
    }

    private String llname_ ;
    private boolean enabled_ = true ;
    private double lastTimestamp = 0.0 ;



    public VisionSubsystem(XeroRobot robot, String llname) {
        super(robot, "vision") ;

        llname_ = llname ;
        enabled_ = true ;
    }

    public void enable() {
        enabled_ = true ;
    }

    public void disable() {
        enabled_ = false ;
    }

    //
    // Note that the odometry stddevs are 0.01, 0.01, 0.002
    //

    @Override
    public void periodic() {
        startPeriodic();

        if (enabled_) {
            CommandSwerveDrivetrain dt = getRobot().getContainer().getDriveBase() ;
            VisionEstimate est = useSimpleApproach() ;
            if (est != null) {
                Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(est.xyStds, est.xyStds, est.thetaStds) ;
                dt.addVisionMeasurement(est.pose, est.timeStamp, visionMeasurementStdDevs) ;
            }
        }

        endPeriodic();
    }

    public SettingsValue getProperty(String name) {
        return null ;
    }

    public Map<String, TalonFX> getCTREMotors() {
        return null ;
    }

    private VisionEstimate useSimpleApproach() {
        VisionEstimate ret = null ;

        PoseEstimate poseest = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llname_);
        if (poseest.timestampSeconds != lastTimestamp) {
            double xyStdDev ;
            double thetaStdDev ;

            // The units here are unclear
            double avgdist = poseest.avgTagDist ;

            if (poseest.tagCount > 1) {
                xyStdDev = 0.001 ;
                thetaStdDev = 9999 ;
            }
            else {
                xyStdDev = 0.0131 * avgdist * avgdist - 0.0278 * avgdist ;
                thetaStdDev = 9999 ;
            }

            ret = new VisionEstimate(poseest.pose, poseest.timestampSeconds, xyStdDev, thetaStdDev) ;
        }

        return ret ;
    }

    private PolynomialRegression xyStdDevModel =
    new PolynomialRegression(
        new double[] {
        0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
        3.223358, 4.093358, 4.726358
        },
        new double[] {0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.12, 0.14, 0.17, 0.27, 0.38},
        2);

private PolynomialRegression thetaStdDevModel =
    new PolynomialRegression(
        new double[] {
        0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
        3.223358, 4.093358, 4.726358
        },
        new double[] {0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068},
        1);

    private VisionEstimate useBreadApproach() {
        VisionEstimate ret = null ;

        PoseEstimate poseest = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llname_);
        if (poseest.timestampSeconds != lastTimestamp) {
            double xyStdDev ;
            double thetaStdDev ;

            // The units here are unclear
            double avgdist = poseest.avgTagDist ;

            if (poseest.tagCount > 1) {
                xyStdDev = Math.pow(avgdist, 2.0) / poseest.tagCount ;
                thetaStdDev = Math.pow(avgdist, 2.0) / poseest.tagCount ;
            }
            else {
                xyStdDev = xyStdDevModel.predict(avgdist) ;
                thetaStdDev = thetaStdDevModel.predict(avgdist) ;
            }

            ret = new VisionEstimate(poseest.pose, poseest.timestampSeconds, xyStdDev, thetaStdDev) ;
        }

        return ret ;
    }
}
