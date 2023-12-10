package org.firstinspires.ftc.teamcode.drive.posePID;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;


@TeleOp(name = "posePIDTest")
public class posePIDTest extends LinearOpMode {
    double x = 0.0, y = 0.0, heading = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain drive = new DriveTrain(hardwareMap);
        stickyGamepad gamepadOne = new stickyGamepad(gamepad1);
        waitForStart();
        while (opModeIsActive()) {
            if(gamepadOne.dpad_up){
                this.x+= 10;
            }else if(gamepadOne.dpad_down){
                this.x-= 10;
            }
            if(gamepadOne.dpad_right){
                this.y+= 10;
            }else if(gamepadOne.dpad_left){
                this.y-= 10;
            }
            if(gamepadOne.right_bumper){
                this.heading+= M.PI/2.0;
            }else if(gamepadOne.left_bumper){
                this.heading-= M.PI/2.0;
            }
            drive.setX(x);
            drive.setY(y);
            drive.setHeading(heading);
            drive.update();
            telemetry.addData("x pos", drive.xPred);
            telemetry.addData("y pos", drive.yPred);
            telemetry.addData("heading", drive.headingPred);
            telemetry.update();
            gamepadOne.update();
        }
    }
}
