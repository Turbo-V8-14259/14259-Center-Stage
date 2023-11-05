package org.firstinspires.ftc.teamcode.opmode.tests.motorTests;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@TeleOp
public class climbTest extends LinearOpMode {
    public DcMotorEx climb;
    @Override
    public void runOpMode() throws InterruptedException {
        climb = hardwareMap.get(DcMotorEx.class, "Pitch");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        while(opModeIsActive()){
            climb.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    gamepad1.right_stick_x
            ));
            telemetry.addData("power draw in amps", climb.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
