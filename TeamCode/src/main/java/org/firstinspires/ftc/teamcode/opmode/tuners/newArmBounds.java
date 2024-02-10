package org.firstinspires.ftc.teamcode.opmode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
@TeleOp
@Disabled
public class newArmBounds extends LinearOpMode {

    Servo turret;
    Servo arm;
    Servo claw;
    stickyGamepad gamepadOne;
    @Override
    public void runOpMode() throws InterruptedException {
        turret = hardwareMap.get(Servo.class, "turret");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0.5);
        gamepadOne = new stickyGamepad(gamepad1);
        waitForStart();
        while(opModeIsActive()){
            if(gamepadOne.dpad_up){
                arm.setPosition(arm.getPosition()+0.05);
            }else if(gamepadOne.dpad_down){
                arm.setPosition(arm.getPosition()-0.05);
            }
            if(gamepadOne.dpad_left){
                turret.setPosition(turret.getPosition()+0.05);
            }else if(gamepadOne.dpad_right){
                turret.setPosition(turret.getPosition()-0.05);
            }
            if(gamepadOne.a){
                claw.setPosition(claw.getPosition()+0.01);
            }else if(gamepadOne.b){
                claw.setPosition(claw.getPosition()-0.01);
            }
            telemetry.addData("arm pos", arm.getPosition()); //.9 lowest, .3 high
            telemetry.addData("turret pos", turret.getPosition()); //0.95 is reset, 0.9
            telemetry.addData("claw pos", claw.getPosition()); //0.5 = latched, 0.47 = unlatched
            telemetry.update();
            gamepadOne.update();
        }


    }
}
