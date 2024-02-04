package org.firstinspires.ftc.teamcode.opmode.tests.motorTests.eachMotorIndividual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;

@TeleOp
public class EndAffector extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        Servo arm = hardwareMap.get(Servo.class, "arm");
        stickyGamepad gamepadOne = new stickyGamepad(gamepad1);
        wrist.setPosition(0);
        arm.setPosition(0);
        waitForStart();
        while(opModeIsActive()){
            if(gamepadOne.dpad_up){
                wrist.setPosition(1);
            }else if(gamepadOne.dpad_down){
                wrist.setPosition(0);
            }
            if(gamepadOne.dpad_left){
                arm.setPosition(1);
            }else if(gamepadOne.dpad_right){
                arm.setPosition(0);
            }
            gamepadOne.update();
            telemetry.addData("wrist",wrist.getPosition());
            telemetry.addData("arm",arm.getPosition());
            telemetry.update();
        }
    }
}
