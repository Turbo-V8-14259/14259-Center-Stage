package org.firstinspires.ftc.teamcode.opmode.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;

@TeleOp
public class depoSlidesTest extends LinearOpMode {
    private DepoSlides slides;
    @Override
    public void runOpMode() throws InterruptedException {
        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class,"leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class,"rightSlides")));
        waitForStart();

        while(opModeIsActive()){
            slides.target = 0;
            slides.update();
            telemetry.addData("position", slides.getCurrentPosition());
            telemetry.addData("target", slides.getTargetPosition());
            telemetry.update();

        }
    }

}
