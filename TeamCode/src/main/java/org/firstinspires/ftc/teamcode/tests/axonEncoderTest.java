package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.usefuls.Sensors.axonEncoder;
@TeleOp
public class axonEncoderTest extends LinearOpMode {
    private axonEncoder bruh;
    AnalogInput encoder;
    CRServo a;

    @Override
    public void runOpMode() throws InterruptedException {


        encoder = hardwareMap.get(AnalogInput.class, "input");
        bruh = new axonEncoder(encoder);
        a = hardwareMap.get(CRServo.class, "axon");
        waitForStart();

        while(opModeIsActive()){
            a.setPower(1);
            telemetry.addData("angle", bruh.update());

            telemetry.update();

        }
    }

}
