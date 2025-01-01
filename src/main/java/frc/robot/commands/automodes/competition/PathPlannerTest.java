package frc.robot.commands.automodes.competition ;

import java.util.Optional;

import org.xero1425.base.XeroAutoCommand;
import org.xero1425.base.XeroRobot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathPlannerTest extends XeroAutoCommand {
    private static final String Path1FileName = "Path1_1" ;
    private static final String Path2FileName = "Path1_2" ;

    private PathPlannerPath p1_ ;
    private PathPlannerPath p2_ ;
    private SequentialCommandGroup series_ ;

    public PathPlannerTest(XeroRobot robot) {
        super(robot, "PathPlannerTest", "Testing back to back paths using path planner") ;

        try {
            p1_ = PathPlannerPath.fromPathFile(Path1FileName) ;
        }
        catch(Exception ex) {
            DriverStation.reportError("Cannot open path file '" + Path1FileName + "' " + ex.getMessage(), true) ;
        }

        try {
            p2_ = PathPlannerPath.fromPathFile(Path2FileName) ;
        }
        catch(Exception ex) {
            DriverStation.reportError("Cannot open path file '" + Path2FileName + "' " + ex.getMessage(), true) ;
        }

        series_ = new SequentialCommandGroup(
            AutoBuilder.followPath(p1_),
            AutoBuilder.followPath(p2_)) ;
    }

    @Override
    public void initialize() {
        Optional<Pose2d> start = p1_.getStartingHolonomicPose() ;
        if (start.isPresent()) {
            getRobot().getContainer().getDriveBase().resetPose(start.get()) ;
        }
        series_.initialize();
    }

    @Override
    public void execute() {
        series_.execute() ;
    }

    @Override
    public void end(boolean interrupted) {
        series_.end(interrupted) ;
    }

    @Override
    public boolean isFinished() {
        return series_.isFinished() ;
    }
}
