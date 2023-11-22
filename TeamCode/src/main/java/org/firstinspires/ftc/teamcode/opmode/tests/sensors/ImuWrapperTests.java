package org.firstinspires.ftc.teamcode.opmode.tests.sensors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.hardware.Sensors.Imu;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;

@TeleOp
@Disabled

public class ImuWrapperTests extends LinearOpMode {
    private Imu imu;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new Imu(hardwareMap.get(IMU.class, "imu"));
        imu.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while(opModeIsActive()){
            
            imu.update();

            telemetry.addData("yaw", imu.getYawR()); //this is the value we're going to be using for centerstage
            telemetry.addData("pitch", imu.getPitchR());
            telemetry.addData("roll", imu.getRollR());
            telemetry.update();
        }
    }
}
