package org.firstinspires.ftc.teamcode.opmode.tests.servoTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;


@TeleOp
public class clawTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.get(Servo.class, "claw");
        stickyGamepad gamepadOne = new stickyGamepad(gamepad1);
        waitForStart();
        while(opModeIsActive()){
            if(gamepadOne.dpad_up){
                claw.setPosition(claw.getPosition()+0.01);
            }else if(gamepadOne.dpad_down){
                claw.setPosition(claw.getPosition()-0.01);
            }
            telemetry.addData("claw",claw.getPosition());
            telemetry.update();
            gamepadOne.update();
        }
    }
}
