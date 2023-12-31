package org.firstinspires.ftc.teamcode.drive.posePID;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;


@TeleOp(name = "xOnly pose pid")
public class xOnly extends LinearOpMode {
    double x = 0.0, y = 0.0, heading = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain drive = new DriveTrain(hardwareMap);
        stickyGamepad gamepadOne = new stickyGamepad(gamepad1);
        waitForStart();
        while (opModeIsActive()) {
            drive.turnYOff();
            drive.turnHeadingOff();
            if(gamepadOne.dpad_up){
                this.x+= 10;
            }else if(gamepadOne.dpad_down){
                this.x-= 10;
            }
            drive.setX(x);
            drive.setY(y);
            drive.setHeading(heading);
            drive.update();
            telemetry.addData("x pos", drive.getX());
            telemetry.addData("y pos", drive.getY());
            telemetry.addData("heading", drive.getHeading());
            telemetry.addData("xPower", drive.getPowerX());
            telemetry.addData("x calculation", drive.getPIDCalculationX());
            telemetry.update();
            gamepadOne.update();
        }
    }
}
