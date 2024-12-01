package org.xero1425.subsystems.oi;

import org.littletonrobotics.junction.AutoLog;

public interface OIInputsOutputs {
    @AutoLog
    public static class OIInputs {
        public boolean buttons_[] = new boolean[32] ;
        public boolean leds_[] = new boolean[16] ;
        public String ledstates_ = "" ;
    }

    public default void updateInputs(OIInputs inputs) {
        //
        // TODO: write me
        //
    }

    public default void updateLEDs() {
        //
        // TODO: write me
        //
    }

    public default void setLEDState(int index, OISubsystem.LEDState st) {
        //
        // TODO: write me
        //
    }
}
