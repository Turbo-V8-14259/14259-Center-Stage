package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Imu;

@TeleOp
public class ImuWrapperTests extends LinearOpMode {
    Imu imu = new Imu();

    double yaw;
    double pitch;
    double roll;

    @Override
    public void runOpMode() throws InterruptedException {

        imu.init();
        waitForStart();

        while(opModeIsActive()){

            yaw = imu.getYawR();
            pitch = imu.getPitchR();
            roll = imu.getRollR();

            telemetry.addData("yaw", yaw);
            telemetry.addData("pitch", pitch);
            telemetry.addData("roll", roll);
            telemetry.update();

        }
    }
}
