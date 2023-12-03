package org.firstinspires.ftc.teamcode.opmode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;

@TeleOp
public class slidesTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DepoSlides slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "rightSlides")));
        slides.stopAndResetEncoder();
        waitForStart();
        while(opModeIsActive()){
            slides.pidRunning = false;
            slides.passive = false;
            slides.manualMode = true;
            slides.setPowerManual(gamepad1.left_trigger - gamepad1.right_trigger);
            telemetry.addData("Slides encoder position ", slides.leftMotor.getCurrentPositionRAW());
            slides.update();
            telemetry.update();
        }
    }
}
