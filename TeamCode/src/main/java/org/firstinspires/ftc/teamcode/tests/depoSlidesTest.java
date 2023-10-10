package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Sensors.axonEncoder;
@TeleOp
public class depoSlidesTest extends LinearOpMode {
    private DepoSlides slides;
    @Override
    public void runOpMode() throws InterruptedException {

        slides = new DepoSlides(hardwareMap.get(DcMotorBetter.class,"leftSlides"), hardwareMap.get(DcMotorBetter.class, "rightSlides"));
        waitForStart();

        while(opModeIsActive()){
            slides.linSlidePosition = 1;
            slides.update();
            telemetry.addData("position", slides.getCurrentPosition());
            telemetry.addData("target", slides.getTargetPosition());
            telemetry.update();

        }
    }

}
