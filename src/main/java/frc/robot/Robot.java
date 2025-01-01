// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.automodes.competition.PathPlannerTest ;

import org.xero1425.base.XeroContainer;
import org.xero1425.base.XeroRobot;
import org.xero1425.simulator.engine.ModelFactory;
import org.xero1425.simulator.engine.SimulationEngine;

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
        //
        // TODO: update the robot container to include the specific subsystems
        //       for this year's robot.
        //
        return new RobotContainer(this) ;
    }

    public String getName() {
        //
        // TODO: change the name of the robot to match this year's robot
        //
        return "TemplateRobot" ;
    }

    @Override
    public String getPracticeSerialNumber() {
        //
        // TODO: if we build two robots, return the serial number for the RoboRio on the
        //       practice bot here.  This will cause the drive base to pick up the parameters
        //       for the practice bot.
        //
        return null;
    }

    @Override
    public void createCompetitionAutoModes() {
        //
        // TODO: add an instance of each auto mode you want to show up in the auto mode
        //       selector on the Shuffleboard or the Elastic display.
        //
        addAutoMode(new PathPlannerTest(this)) ;
    }

    @Override
    public String getSimulationFileName() {
        //
        // TODO: return the name of the simulation file to run
        //
        return "automode" ;
    }

    @Override
    public String getSimulationAutoMode() {
        //
        // TODO: return the name of the auto mode to run while simulating
        //
        return "PathPlannerTest" ;
    }

    @Override
    public void addRobotSimulationModels() {
        //
        // TODO: add any simulation models that simulate this years robot subsystems
        //
        ModelFactory factory = SimulationEngine.getInstance().getModelFactory();
        factory.registerModel("robot-oi", "frc.models.RobotOIModel");         
    }
}
