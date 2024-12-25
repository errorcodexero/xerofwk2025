package org.xero1425.subsystems.vision;

import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.xero1425.base.LimelightHelpers;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroSubsystem;
import org.xero1425.base.LimelightHelpers.PoseEstimate;
import org.xero1425.misc.SettingsValue;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.VisionConstants;

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
    private boolean enabled_ ;
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

    public boolean isEnabled() {
        return enabled_ ;
    }

    //
    // Note that the odometry stddevs are 0.01, 0.01, 0.002
    //

    @Override
    public void periodic() {
        startPeriodic();

        if (enabled_) {
            CommandSwerveDrivetrain dt = getRobot().getContainer().getDriveBase() ;

            Rotation2d robotHeading = dt.getState().Pose.getRotation();
            LimelightHelpers.SetRobotOrientation(llname_, robotHeading.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0) ;

            VisionEstimate est = useSimpleApproach() ;
            if (est != null && est.pose.getTranslation().getNorm() != 0) {
                Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(est.xyStds, est.xyStds, est.thetaStds) ;
                double ts = Utils.fpgaToCurrentTime(est.timeStamp) ;
                dt.addVisionMeasurement(est.pose, ts, visionMeasurementStdDevs) ;
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
        if (poseest != null) {
            Logger.recordOutput("limelight/pose", poseest.pose) ;
        }
        if (poseest != null && poseest.timestampSeconds != lastTimestamp) {

            double xyStdDev ;
            double thetaStdDev ;

            if (poseest.tagCount != 0) {
                if (poseest.tagCount > 1) {
                    xyStdDev = VisionConstants.MultiTagXyStdDev ;
                    thetaStdDev = VisionConstants.MultiTagThetaStdDev ;
                }
                else {
                    //
                    // TODO: Here most good teams have a quadratic equation mapping the tag distance
                    //       to the standard deviation value.
                    //
                    xyStdDev = VisionConstants.SingleTagXyStdDev ;
                    thetaStdDev = VisionConstants.SingleTagThetaStdDev ;
                }

                ret = new VisionEstimate(poseest.pose, poseest.timestampSeconds, xyStdDev, thetaStdDev) ;
                lastTimestamp = poseest.timestampSeconds;
            }
        }

        return ret ;
    }
}
