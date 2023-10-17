package org.firstinspires.ftc.teamcode.opmode.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.Sensors.axonEncoder;
@TeleOp
public class axonEncoderTest extends LinearOpMode {
    private axonEncoder test;
    private AnalogInput encoder;
    private CRServo a;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();
        waitForStart();

        while(opModeIsActive()){
            a.setPower(.02);
            telemetry.addData("angle", test.update());

            telemetry.update();
        }
    }

    private void initializeComponents() {
        encoder = hardwareMap.get(AnalogInput.class, "input");
        test = new axonEncoder(encoder);
        a = hardwareMap.get(CRServo.class, "axon");
    }

}
