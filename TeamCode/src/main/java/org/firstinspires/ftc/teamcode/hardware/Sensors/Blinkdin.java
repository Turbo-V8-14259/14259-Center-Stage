package org.firstinspires.ftc.teamcode.hardware.Sensors;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

public class Blinkdin {

    RevBlinkinLedDriver.BlinkinPattern target;
    RevBlinkinLedDriver led;

    public Blinkdin(RevBlinkinLedDriver led){
        this.led = led;
    }

    public Blinkdin changePattern(RevBlinkinLedDriver.BlinkinPattern setting){
        this.target = setting;
        return this;
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern(){
        return target;
    }
    public void update(){
        this.led.setPattern(target);
    }

}
