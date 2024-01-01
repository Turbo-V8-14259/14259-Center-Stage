package org.firstinspires.ftc.teamcode.drive.posePID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;


@TeleOp(name = "rotationalOnly pose pid")
@Config
@Disabled

public class rotationalOnly extends LinearOpMode {
    public static double x = 0.0, y = 0.0, heading = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DriveTrain drive = new DriveTrain(hardwareMap);
        stickyGamepad gamepadOne = new stickyGamepad(gamepad1);
        waitForStart();
        while (opModeIsActive()) {
            drive.turnYOff();
            drive.turnXOff();
            if(gamepadOne.right_bumper){
                this.heading+= M.PI/2.0;
            }else if(gamepadOne.left_bumper){
                this.heading-= M.PI/2.0;
            }
            drive.setX(x);
            drive.setY(y);
            drive.setHeading(heading);
            drive.update();
            telemetry.addData("x pos", drive.getX());
            telemetry.addData("y pos", drive.getY());
            telemetry.addData("heading", drive.getHeading());
            telemetry.addData("headingPower", drive.getPowerHeading());
            telemetry.addData("r calculation", drive.getPIDCalculationHeading());
            telemetry.update();
            gamepadOne.update();
        }
    }
}
