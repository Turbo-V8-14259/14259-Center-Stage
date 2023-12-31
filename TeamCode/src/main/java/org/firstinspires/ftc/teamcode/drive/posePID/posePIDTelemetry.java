package org.firstinspires.ftc.teamcode.drive.posePID;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;


@TeleOp(name = "posePID telemetry")
public class posePIDTelemetry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
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
            telemetry.update();
        }
    }
}
