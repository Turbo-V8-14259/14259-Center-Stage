package org.firstinspires.ftc.teamcode.opmode.tests.motorTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class motorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor e = hardwareMap.get(DcMotor.class,"e");
        waitForStart();
        while(opModeIsActive()){
            e.setPower(.5);
        }
    }
}
