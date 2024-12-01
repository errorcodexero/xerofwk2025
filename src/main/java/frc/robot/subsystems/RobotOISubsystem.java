package frc.robot.subsystems;

import org.xero1425.base.XeroRobot;
import org.xero1425.subsystems.oi.OISubsystem;
import frc.robot.constants.OIConstants;

//
// This is a template for an OI class
//

public class RobotOISubsystem extends OISubsystem {
    public RobotOISubsystem(XeroRobot robot) {
        super(robot, "OI", OIConstants.kDriverControllerPort, OIConstants.kOIControllerPort) ;
    }
}
