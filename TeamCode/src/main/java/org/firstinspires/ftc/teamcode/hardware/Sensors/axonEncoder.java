package org.firstinspires.ftc.teamcode.hardware.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class axonEncoder{

    private AnalogInput analog;

    private double angle = 0;

    public axonEncoder(AnalogInput analog) {
        this.analog = analog;
    }

    public double getVoltage(){
        return analog.getVoltage();
    }
    public double update() {
        angle = getVoltage()/3.3*360;
        return angle;
    }
}
