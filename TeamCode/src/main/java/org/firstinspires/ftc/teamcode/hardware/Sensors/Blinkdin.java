//package org.firstinspires.ftc.teamcode.hardware.Sensors;
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
//import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;
//
//public class Blinkdin {
//
//    double target = 0;
//    ServoMotorBetter led;
//
//    public Blinkdin(ServoMotorBetter led){
//        this.led = led;
//        this.led.setLowerBound(0.2525);
//        this.led.setUpperBound(.7475);
//    }
//
//    public Blinkdin changePattern(double setting){
//        this.target = setting;
//        return this;
//    }
//
//    public void update(){
//        this.led.setPosition(target);
//        this.led.update();
//    }
//
//}
