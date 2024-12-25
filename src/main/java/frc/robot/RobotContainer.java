package frc.robot;

import org.xero1425.base.XeroContainer;
import org.xero1425.base.XeroRobot;
import org.xero1425.subsystems.vision.VisionSubsystem;

import frc.robot.subsystems.RobotOISubsystem;

public class RobotContainer extends XeroContainer {
    private VisionSubsystem vision_ ;

    public RobotContainer(XeroRobot robot) {
        //
        // This XeroContainer constructor will create the swerve drive
        // base.  Tie constructor will also bind the swerve drive base to
        // the gamepad.
        //
        super(robot, new RobotOISubsystem(robot)) ;

        vision_ = new VisionSubsystem(robot, "") ;
    }

    public VisionSubsystem getVision() {
        return vision_ ;
    }
}
