package org.xero1425.base;

import java.util.Optional;

import org.xero1425.subsystems.oi.OISubsystem;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;
import org.xero1425.subsystems.swerve.Telemetry;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import static edu.wpi.first.units.Units.*;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.TunerConstantsCompBot;
import frc.robot.constants.TunerConstantsPracticeBot;

public class XeroContainer {
    private static LinearVelocity MaxSpeed = MetersPerSecond.of(4.61) ;
    private static AngularVelocity MaxAngularRate = DegreesPerSecond.of(720) ;

    private XeroRobot robot_;

    //
    // OI related members
    //
    private OISubsystem oi_;

    //
    // Swerve drive related members
    //
    private Telemetry telemetry_;
    private CommandSwerveDrivetrain db_;
    private SwerveRequest.FieldCentric drive_;

    public XeroContainer(XeroRobot robot, OISubsystem oi) {
        robot_ = robot;

        oi_ = oi ;
        createDriveBase();
        dbBindings();
    }

    public OISubsystem getOISubsystem() {
        return oi_;
    }

    public CommandSwerveDrivetrain getDriveBase() {
        return db_;
    }

    private void createDriveBase() {
        if (robot_.isPracticeBot()) {
            //
            // Create the drivebase
            //
            Matrix<N3, N1> odomStdDev = VecBuilder.fill(0.1, 0.1, 0.1) ;
            Matrix<N3, N1> visionStdDev = VecBuilder.fill(0.1, 0.1, 0.1) ;
            db_ = new CommandSwerveDrivetrain(
                TunerConstantsPracticeBot.DrivetrainConstants, 0,
                    odomStdDev, visionStdDev,                     
                    TunerConstantsPracticeBot.FrontLeft,
                    TunerConstantsPracticeBot.FrontRight,
                    TunerConstantsPracticeBot.BackLeft,
                    TunerConstantsPracticeBot.BackRight);                    

            telemetry_ = new Telemetry(TunerConstantsCompBot.kSpeedAt12Volts.magnitude());

            drive_ = new SwerveRequest.FieldCentric()
                    .withDeadband(TunerConstantsCompBot.kSpeedAt12Volts.magnitude() * 0.05)
                    .withRotationalDeadband(SwerveConstants.kMaxRotationalSpeed * 0.05)
                    .withDriveRequestType(DriveRequestType.Velocity);
        } else {

            Matrix<N3, N1> odomStdDev = VecBuilder.fill(0.1, 0.1, 0.1) ;
            Matrix<N3, N1> visionStdDev = VecBuilder.fill(0.1, 0.1, 0.1) ;
            db_ = new CommandSwerveDrivetrain(
                    TunerConstantsCompBot.DrivetrainConstants, 0,
                    odomStdDev, visionStdDev,                     
                    TunerConstantsCompBot.FrontLeft,
                    TunerConstantsCompBot.FrontRight,
                    TunerConstantsCompBot.BackLeft,
                    TunerConstantsCompBot.BackRight);

            telemetry_ = new Telemetry(TunerConstantsCompBot.kSpeedAt12Volts.magnitude());

            drive_ = new SwerveRequest.FieldCentric()
                    .withDeadband(TunerConstantsCompBot.kSpeedAt12Volts.magnitude() * 0.05)
                    .withRotationalDeadband(SwerveConstants.kMaxRotationalSpeed * 0.05)
                    .withDriveRequestType(DriveRequestType.Velocity);
        }

        db_.registerTelemetry(telemetry_::telemeterize);
    }

    private void yandbPressed() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            Pose2d pose;
            if (alliance.get() == Alliance.Red) {
                pose = new Pose2d(0, 0, Rotation2d.fromDegrees(180.0));
            } else {
                pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0.0));
            }
            db_.resetPose(pose);
        } else {
            DriverStation.reportError("Gamepad Y & B pressed before alliance is known (should be impossible)", false);
        }
    }    

    protected void dbBindings() {
        //
        // Create the standard bindings between the gamepad and the drive base
        //
        db_.setDefaultCommand(
                db_.applyRequest(() -> drive_
                        .withVelocityX(MaxSpeed.times(-oi_.getLeftY()))
                        .withVelocityY(MaxSpeed.times(-oi_.getLeftX()))
                        .withRotationalRate(MaxAngularRate.times(-oi_.getRightX()))
                )
        );

        oi_.getXBoxController().y().and(oi_.getXBoxController().b())
                .onTrue(db_.runOnce(() -> yandbPressed()).ignoringDisable(true));        
    }
}
