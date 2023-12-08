package org.firstinspires.ftc.teamcode.opmode.tests.sensors;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.hardware.Sensors.Blinkdin;


@TeleOp
public class blinkdinTest extends LinearOpMode {
    Blinkdin led;
    @Override
    public void runOpMode() throws InterruptedException {
        led = new Blinkdin(hardwareMap.get(RevBlinkinLedDriver.class, "led"));
        while(opModeInInit()){
            led.changePattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            led.update();
        }
        while(opModeIsActive()){
            led.changePattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            led.update();
        }

    }
}
