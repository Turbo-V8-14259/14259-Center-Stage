package org.firstinspires.ftc.teamcode.opmode.tests.motorTests.eachMotorIndividual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SLIDES MOTORS")
public class depositSlideOne extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor sL = hardwareMap.get(DcMotor.class, "leftSlides");
        DcMotor sR = hardwareMap.get(DcMotor.class, "rightSlides");
        sR.setDirection(DcMotorSimple.Direction.REVERSE);
        sL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            sL.setPower(gamepad1.left_stick_x);
            sR.setPower(gamepad1.left_stick_x);
            telemetry.addData("sL power ", sL.getPower());
            telemetry.addData("sR power ", sR.getPower());
            telemetry.addData("SR position ", sR.getCurrentPosition());
            telemetry.addData("SL position ", sL.getCurrentPosition());
            telemetry.update();
        }
    }
}
