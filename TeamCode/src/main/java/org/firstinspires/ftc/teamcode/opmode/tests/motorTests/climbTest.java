package org.firstinspires.ftc.teamcode.opmode.tests.motorTests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class climbTest extends LinearOpMode {
    public DcMotorEx climb;
    @Override
    public void runOpMode() throws InterruptedException {
        climb = hardwareMap.get(DcMotorEx.class, "climb");
        waitForStart();
        while(opModeIsActive()){
            climb.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
        }
    }
}
