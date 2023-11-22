package org.firstinspires.ftc.teamcode.opmode.tests.sensors;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
@Disabled
public class oldIMU extends LinearOpMode {
    private BNO055IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("imu angle", imu.getAngularOrientation().firstAngle);
            telemetry.update();
        }

    }
}
