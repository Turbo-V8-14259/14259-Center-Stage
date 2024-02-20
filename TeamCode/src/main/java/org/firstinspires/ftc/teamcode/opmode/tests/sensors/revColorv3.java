package org.firstinspires.ftc.teamcode.opmode.tests.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
@TeleOp
public class revColorv3 extends LinearOpMode {
    ColorSensor color;
    ColorSensor color2;

    @Override
    public void runOpMode() throws InterruptedException {
        color = hardwareMap.get(ColorSensor.class, "Color1");
        color2 = hardwareMap.get(ColorSensor.class, "Color2");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("color 1 green ",color.green());
            telemetry.addData("color 2 green ",color2.green());
            telemetry.update();
        }
    }
}
