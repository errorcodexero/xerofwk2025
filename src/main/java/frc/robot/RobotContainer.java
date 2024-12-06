package frc.robot;

import org.xero1425.base.XeroContainer;
import org.xero1425.base.XeroRobot;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotOISubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer extends XeroContainer {
    private IntakeSubsystem intake_ ;
    private VisionSubsystem vision_ ;

    public RobotContainer(XeroRobot robot) {
        //
        // This XeroContainer constructor will create the swerve drive
        // base.  Tie constructor will also bind the swerve drive base to
        // the gamepad.
        //
        super(robot, new RobotOISubsystem(robot)) ;

        vision_ = new VisionSubsystem(robot, "") ;
        intake_ = new IntakeSubsystem(robot) ;
    }

    public IntakeSubsystem getIntake() {
        return intake_ ;
    }

    public VisionSubsystem getVision() {
        return vision_ ;
    }
}
