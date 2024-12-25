// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.xero1425.base.XeroContainer;
import org.xero1425.base.XeroRobot;
import org.xero1425.simulator.engine.ModelFactory;
import org.xero1425.simulator.engine.SimulationEngine;
import frc.robot.commands.automodes.competition.DriveStraight;
import frc.robot.commands.automodes.competition.VisionTest;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends XeroRobot {
    private final static boolean kLogToNetworkTables = true ;

    public Robot() {
        super(kLogToNetworkTables) ;
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        super.robotInit() ;
    }

    @Override
    public XeroContainer createContainer() {
        return new RobotContainer(this) ;
    }

    public String getName() {
        return "TemplateRobot" ;
    }

    public boolean isCharMode() {
        return false ;
    }

    @Override
    public String getPracticeSerialNumber() {
        return null;
    }

    @Override
    public void createCompetitionAutoModes() {
        addAutoMode(new VisionTest(this)) ;
        addAutoMode(new DriveStraight(this));
    }

    @Override
    public String getSimulationFileName() {
        return "oiled" ;
    }

    @Override
    public String getSimulationAutoMode() {
        return "LEDTest" ;
    }

    @Override
    public void addRobotSimulationModels() {
        ModelFactory factory = SimulationEngine.getInstance().getModelFactory();
        factory.registerModel("robot-oi", "frc.models.RobotOIModel");         
    }
}
