package frc.robot;

import org.xero1425.base.XeroContainer;
import org.xero1425.base.XeroRobot;

import frc.robot.subsystems.RobotOISubsystem;

public class RobotContainer extends XeroContainer {
    public RobotContainer(XeroRobot robot) {
        //
        // This XeroContainer constructor will create the swerve drive
        // base.  Tie constructor will also bind the swerve drive base to
        // the gamepad.
        //
        super(robot, new RobotOISubsystem(robot)) ;

        //
        // TODO: create additional subsystems here
        //
    }
}
