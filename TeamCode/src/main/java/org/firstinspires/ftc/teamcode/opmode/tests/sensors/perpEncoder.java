package org.firstinspires.ftc.teamcode.opmode.tests.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.util.Encoder;


@TeleOp
public class perpEncoder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx perpendicularEncoder = (hardwareMap.get(DcMotorEx.class, "RightBack"));
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("perpen", perpendicularEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
