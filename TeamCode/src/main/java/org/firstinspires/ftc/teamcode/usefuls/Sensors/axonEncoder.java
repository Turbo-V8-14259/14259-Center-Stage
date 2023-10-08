package org.firstinspires.ftc.teamcode.usefuls.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class axonEncoder{

    private AnalogInput analog;
    private double voltage;

    private double angle = 0;

    public axonEncoder(AnalogInput analog) {
        this.analog = analog;
        this.voltage = getVoltage();
    }


    public double getVoltage() {
        voltage = this.analog.getVoltage();
        return 0;
    }

    public double update() {
        angle = voltage/3.3*360;
        return angle;
    }
}
