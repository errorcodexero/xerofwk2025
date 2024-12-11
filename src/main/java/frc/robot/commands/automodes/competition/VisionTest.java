package frc.robot.commands.automodes.competition;

import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;
import org.xero1425.math.Pose2dWithRotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;

public class VisionTest extends XeroAutoCommand {
    
    private final static String desc = "This auto mode is used to test vision accuracy" ;
    private final static double max1v = 1.0 ;
    private final static double max1a = 1.0 ;
    private final static double max2v = 1.0 ;
    private final static double max2a = 1.0 ;

    private Pose2dWithRotation dest1_ ;
    private Pose2dWithRotation dest2_ ;
    private boolean first_ ;
    private boolean done_ ;
    
    public VisionTest(XeroRobot robot) {
        super(robot, "vision-test", desc) ;
        addRequirements(getRobot().getContainer().getDriveBase()) ;
    }

    @Override
    public void initialize() {
        first_ = true;

        RobotContainer container = (RobotContainer)getRobot().getContainer();
        container.getVision().enable() ;

        dest1_ = new Pose2dWithRotation(7.78, 7.48, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) ;
        dest2_ = new Pose2dWithRotation(2.5, 5.5, Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)) ;

        getRobot().getContainer().getDriveBase().resetPose(new Pose2d(1.3, 5.77, Rotation2d.fromDegrees(0.0))) ;

        //
        // Drive the first path
        //
        getRobot().getContainer().getDriveBase().driveTo("test", null, dest1_, max1v, max1a, 
                                                         0.0, 0.0, 5.0) ;
    }

    @Override
    public void execute() {
        super.execute() ;

        if (!getRobot().getContainer().getDriveBase().isFollowingPath()) {
            if (first_) {
                getRobot().getContainer().getDriveBase().driveTo("test", null, dest2_, max2v, max2a, 
                                                                0.0, 0.0, 0.0) ;
                first_ = false ;
            }
            else {
                RobotContainer container = (RobotContainer)getRobot().getContainer();
                container.getVision().disable() ;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        getRobot().getContainer().getDriveBase().stopPath() ;
    }

    @Override
    public boolean isFinished() {
        return done_ ;
    }
}
