package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class Blinkdin {

    public RevBlinkinLedDriver led;

    public RevBlinkinLedDriver.BlinkinPattern pattern;

    public Blinkdin(RevBlinkinLedDriver led){
        this.led = led;
    }

    public Blinkdin changePattern(RevBlinkinLedDriver.BlinkinPattern newPattern){
        this.pattern = newPattern;
    }

    public void update(){
        this.led.setPattern(pattern);
    }

}
