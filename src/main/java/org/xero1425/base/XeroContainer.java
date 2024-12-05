package org.xero1425.base;

import java.util.Optional;

import org.xero1425.subsystems.oi.OISubsystem;
import org.xero1425.subsystems.swerve.CommandSwerveDrivetrain;
import org.xero1425.subsystems.swerve.Telemetry;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.TunerConstantsCompBot;
import frc.robot.constants.TunerConstantsPracticeBot;

public class XeroContainer {
    private static double MaxSpeed = 4.61 ;
    private static double MaxAngularRate = 720;

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
            db_ = new CommandSwerveDrivetrain(
                    TunerConstantsPracticeBot.DrivetrainConstants,
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
            //
            // Create the drivebase
            //
            db_ = new CommandSwerveDrivetrain(
                    TunerConstantsCompBot.DrivetrainConstants,
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
                        .withVelocityX(-oi_.getLeftY() * MaxSpeed)
                        .withVelocityY(-oi_.getLeftX() * MaxSpeed)
                        .withRotationalRate(-oi_.getRightX() * MaxAngularRate)
                )
        );

        oi_.getXBoxController().y().and(oi_.getXBoxController().b())
                .onTrue(db_.runOnce(() -> yandbPressed()).ignoringDisable(true));        
    }
}
