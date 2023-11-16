package org.firstinspires.ftc.teamcode.opmode.tests.motorTests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
@Config
@TeleOp
@Disabled

public class depoSlidesTest extends LinearOpMode {
    public static double targetSet = 0;
    private DepoSlides slides;
    stickyGamepad gamepadOne;

    @Override
    public void runOpMode() throws InterruptedException {

        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class,"leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class,"rightSlides")));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadOne = new stickyGamepad(gamepad1);
        waitForStart();

        while(opModeIsActive()){
            if(gamepadOne.dpad_up){
                targetSet+=5;
            }
            if(gamepadOne.dpad_down){
                targetSet-=5;
            }
            slides.setInches(targetSet);
            slides.update();
            telemetry.addData("position in inches", slides.getCurrentInches());
            telemetry.addData("target", targetSet);
            telemetry.update();
            gamepadOne.update();
        }
    }

}
