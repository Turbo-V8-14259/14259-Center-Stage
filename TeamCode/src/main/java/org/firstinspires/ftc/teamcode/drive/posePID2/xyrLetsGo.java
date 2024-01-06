package org.firstinspires.ftc.teamcode.drive.posePID2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "pose pid tuner")
@Config

public class xyrLetsGo extends LinearOpMode {
    public static double y = 0.0, x = 0.0, r = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        DT drive = new DT(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()){
            drive.setYTarget(y);
            drive.setXTarget(x);
            drive.setRTarget(Math.toRadians(r));
            drive.update();
            telemetry.addData("delta x", drive.getX() - x));
            telemetry.addData("delta y ", Math.abs(drive.getY() - y));
            telemetry.addData("delta theta deg ", Math.abs(Math.toDegrees(drive.getR()) - r));
            telemetry.update();
        }
    }
}
