package frc.robot.subsystems;

import java.util.Map;

import org.xero1425.base.TalonFXFactory;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroSubsystem;
import org.xero1425.misc.SettingsValue;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem extends XeroSubsystem {
    private TalonFX feeder_ ;
    private TalonFX tilt_ ;
    private TalonFX updown_ ;

    public IntakeSubsystem(XeroRobot robot) {
        super(robot,"intake") ;

        try {
            //
            // This just keeps these motors in brake mode so they don't 
            // bounce around.
            //
            feeder_ = TalonFXFactory.getFactory().createTalonFX(1, false) ;
            tilt_ = TalonFXFactory.getFactory().createTalonFX(5, false) ;
            updown_ = TalonFXFactory.getFactory().createTalonFX(2, false) ;
        }
        catch(Exception ex) {

        }
    }

    public SettingsValue getProperty(String name) {
        return null ;
    }

    public Map<String, TalonFX> getCTREMotors() {
        return null ;
    }
}
