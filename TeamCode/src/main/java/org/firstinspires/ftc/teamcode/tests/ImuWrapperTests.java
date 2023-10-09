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
            imu.update();

            telemetry.addData("yaw", imu.getYawR());
            telemetry.addData("pitch", imu.getPitchR());
            telemetry.addData("roll", imu.getRollR());
            telemetry.update();

        }
    }
}
