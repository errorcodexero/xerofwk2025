package org.xero1425.base;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.xero1425.misc.SimArgs;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.subsystems.oi.OISubsystem;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public abstract class XeroRobot extends LoggedRobot {

    private static XeroRobot robot_ = null;

    private XeroAutoCommand auto_mode_;
    private List<XeroAutoCommand> automodes_;
    private SendableChooser<XeroAutoCommand> chooser_;
    private GenericEntry chosen_;
    private GenericEntry desc_;
    private boolean auto_modes_created_;

    private XeroContainer container_ ;

    protected OISubsystem oi_;

    private HashMap<String, Double> periodic_times_;
    private HashMap<String, ISubsystemSim> subsystems_;

    public XeroRobot(boolean logToNetworkTables) {
        if (robot_ != null) {
            throw new RuntimeException("XeroRobot is a singleton class");
        }
        robot_ = this;
        subsystems_ = new HashMap<>();
        periodic_times_ = new HashMap<>();

        auto_modes_created_ = false;
        automodes_ = new ArrayList<>();
        auto_mode_ = null;

        enableAdvantageKitLogger(logToNetworkTables);

        if (RobotBase.isSimulation()) {
            String str = SimArgs.InputFileName;
            if (str == null)
                str = getSimulationFileName();

            if (str == null) {
                System.out.println(
                        "The code is setup to simulate, but the derived robot class did not provide a stimulus file");
                System.out.println("Not initializing the Xero1425 Simulation engine - assuming Romi robot");
            } else {
                SimulationEngine.initializeSimulator(this);
                addRobotSimulationModels();
                SimulationEngine.getInstance().initAll(str);
            }
        }
    }

    public abstract boolean isCharMode();

    public abstract String getSimulationFileName();

    public abstract String getSimulationAutoMode();

    protected abstract String getName();

    protected abstract String getPracticeSerialNumber();

    protected abstract void createCompetitionAutoModes();

    protected abstract void addRobotSimulationModels();

    protected abstract void robotSpecificBindings();

    protected abstract String getCharSubsystem();

    protected abstract String getCharMotor();

    protected abstract XeroContainer createContainer() ;

    public XeroContainer getContainer() {
        return container_ ;
    }

    public void startPeriodic(String name) {
        periodic_times_.put(name, Timer.getFPGATimestamp());
    }

    public void endPeriodic(String name) {
        double runtime = Double.NaN;
        if (periodic_times_.containsKey(name)) {
            runtime = Timer.getFPGATimestamp() - periodic_times_.get(name);
        }

        Logger.recordOutput("timingsMS:" + name, 1000 * runtime);
    }

    public ISubsystemSim getSubsystemByName(String name) {
        return subsystems_.get(name);
    }

    public void robotInit() {
        super.robotInit();

        if (RobotBase.isSimulation() && SimulationEngine.getInstance() != null) {
            //
            // If we are simulating, create the simulation modules required
            //
            SimulationEngine.getInstance().createModels();
        }

        container_ = createContainer() ;
    }

    protected void createAutoModes() {
        if (DriverStation.isDSAttached()) {
            if (!auto_modes_created_) {
                createCompetitionAutoModes();
                auto_modes_created_ = true;
            }

            autoModeChooser();
        }
    }

    protected void addAutoMode(XeroAutoCommand mode) {
        automodes_.add(mode);
    }

    private void enableAdvantageKitLogger(boolean logToNetworkTables) {
        Logger.disableDeterministicTimestamps();

        if (XeroRobot.isSimulation() || logToNetworkTables) {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.start();
    }

    @Override
    public void robotPeriodic() {
        //
        // Runs the Scheduler.
        //
        CommandScheduler.getInstance().run();

        if (isSimulation()) {
            SimulationEngine engine = SimulationEngine.getInstance();
            if (engine != null) {
                engine.run(getPeriod());
            }
        }

    }

    public void registerSubsystem(String name, ISubsystemSim subsystem) {
        subsystems_.put(name, subsystem);
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();

        if (auto_mode_ != null) {
            Logger.recordMetadata("auto-mode-run", auto_mode_.getName());
            auto_mode_.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void disabledPeriodic() {
        createAutoModes();
    }

    protected void enableMessages() {
    }

    public boolean isPracticeBot() {
        return RobotController.getSerialNumber().equals(getPracticeSerialNumber());
    }

    public boolean isCompetitionBot() {
        return RobotBase.isReal() && !isPracticeBot();
    }

    public double getTime() {
        return Timer.getFPGATimestamp();
    }

    private void autoModeChanged(XeroAutoCommand mode) {
        chosen_.setString(mode.toString());
        desc_.setString(mode.getDescription());
    }

    private void autoModeChooser() {
        if (XeroRobot.isSimulation()) {
            String name = getSimulationAutoMode();
            for (XeroAutoCommand cmd : automodes_) {
                if (cmd.getName().equals(name)) {
                    auto_mode_ = cmd;
                    break;
                }
            }
        } else {
            if (chooser_ == null && automodes_.size() > 0) {
                chooser_ = new SendableChooser<>();
                chooser_.onChange((mode) -> autoModeChanged(mode));
                boolean defaultSet = false;
                for (XeroAutoCommand mode : automodes_) {
                    chooser_.addOption(mode.toString(), mode);
                    if (!defaultSet) {
                        auto_mode_ = mode;
                        chooser_.setDefaultOption(mode.toString(), mode);
                        defaultSet = true;
                    }
                }

                ShuffleboardTab tab = Shuffleboard.getTab("AutoMode");
                chosen_ = tab.add("Auto Mode Selected", auto_mode_.toString()).withSize(2, 1).withPosition(3, 0)
                        .getEntry();
                desc_ = tab.add("Auto Mode Description", auto_mode_.toString()).withSize(5, 2).withPosition(0, 1)
                        .getEntry();
                tab.add("Auto Mode Selecter", chooser_).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser)
                        .withPosition(0, 0);
            }

            if (chooser_ != null) {
                auto_mode_ = chooser_.getSelected();
            }
        }
    }
}
