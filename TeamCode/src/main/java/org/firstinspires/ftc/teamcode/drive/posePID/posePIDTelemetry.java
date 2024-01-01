package org.firstinspires.ftc.teamcode.drive.posePID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "posePID ALLO?")
@Disabled

public class posePIDTelemetry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DriveTrain drive = new DriveTrain(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            drive.turnYOff();
            drive.turnHeadingOff();
            drive.turnXOff();
            drive.update();
            telemetry.addData("x pos", drive.getX());
            telemetry.addData("y pos", drive.getY());
            telemetry.addData("heading", drive.getHeading());
            telemetry.addData("xPower", drive.getPowerX());
            telemetry.addData("yPower", drive.getPowerY());
            telemetry.addData("headingPower", drive.getPowerHeading());
            telemetry.addData("x calculation", drive.getPIDCalculationX());
            telemetry.addData("y calculation", drive.getPIDCalculationY());
            telemetry.addData("r calculation", drive.getPIDCalculationHeading());
            telemetry.update();
        }
    }
}
