package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.hardware.Sensors.goBildaTouchDriver;

@TeleOp
public class gobildaTouchSample extends LinearOpMode {
    private goBildaTouchDriver sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = new goBildaTouchDriver(hardwareMap.get(DigitalChannel.class, "touch"));

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("touch", sensor.check());
            telemetry.update();
        }
    }
}
