package org.firstinspires.ftc.teamcode.opmode.tests.motorTests;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
@Disabled

public class climbTest extends LinearOpMode {
    public DcMotorEx climb;
    @Override
    public void runOpMode() throws InterruptedException {
        climb = hardwareMap.get(DcMotorEx.class, "Pitch");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            climb.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();
            telemetry.addData("power draw in amps", climb.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
