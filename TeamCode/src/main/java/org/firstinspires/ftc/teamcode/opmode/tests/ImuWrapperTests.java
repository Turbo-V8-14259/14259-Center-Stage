package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.hardware.Sensors.Imu;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;

@TeleOp
public class ImuWrapperTests extends LinearOpMode {
    private Imu imu;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new Imu(hardwareMap.get(IMU.class, "imu"));
        imu.init();

        waitForStart();

        while(opModeIsActive()){
            
            imu.update();

            telemetry.addData("yaw", M.toDegrees(imu.getYawR()));
            telemetry.addData("pitch", M.toDegrees(imu.getPitchR()));
            telemetry.addData("roll", M.toDegrees(imu.getRollR()));
            telemetry.update();
        }
    }
}
