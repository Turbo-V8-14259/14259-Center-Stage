package org.firstinspires.ftc.teamcode.opmode.tests.motorTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Disabled

public class slidesTestStandalone extends LinearOpMode {
    double power = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx slidesLeft = hardwareMap.get(DcMotorEx.class, "leftSlides");
        DcMotorEx slidesRight = hardwareMap.get(DcMotorEx.class, "rightSlides");

        slidesRight.setDirection(DcMotorEx.Direction.REVERSE);
        slidesLeft.setDirection(DcMotorEx.Direction.FORWARD);
        //change to reverse if needed
        waitForStart();
        while(opModeIsActive()){
            power = gamepad1.left_trigger - gamepad1.right_trigger;
            slidesRight.setPower(power);
            slidesLeft.setPower(power);
        }
    }
}
