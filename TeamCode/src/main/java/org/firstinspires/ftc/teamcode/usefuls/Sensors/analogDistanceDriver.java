package org.firstinspires.ftc.teamcode.usefuls.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class analogDistanceDriver {

    private final double maxR;
    private AnalogInput analog;
    private double voltage;
//    private int maxR = 520;

    public analogDistanceDriver(AnalogInput analog, double maxR) {
        this.analog = analog;
        this.maxR = maxR;
    }

    public double getDistance() {
        voltage = this.analog.getVoltage();
        double distance = (voltage*this.maxR)/3.3;
        return distance;
    }


}


