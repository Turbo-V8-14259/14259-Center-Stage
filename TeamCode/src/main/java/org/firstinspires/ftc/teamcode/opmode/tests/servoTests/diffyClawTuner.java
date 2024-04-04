package org.firstinspires.ftc.teamcode.opmode.tests.servoTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class diffyClawTuner extends LinearOpMode {
    Servo diffy1;
    Servo diffy2;

    double diffy1Pos = 0;
    double diffy2Pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        diffy1 = hardwareMap.get(Servo.class, "name1");
        diffy2 = hardwareMap.get(Servo.class, "name2");
        diffy1.setPosition(0);
        diffy2.setPosition(0);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_up){
                diffy1Pos+=.05;
            }else if(gamepad1.dpad_down){
                diffy1Pos-=.05;
            }
            if(gamepad1.dpad_left){
                diffy2Pos+=.05;
            }else if(gamepad1.dpad_right){
                diffy2Pos-=0.5;
            }
            if(diffy1Pos > 1)
                diffy1Pos = 1;
            if(diffy1Pos < 0)
                diffy1Pos = 0;
            if(diffy2Pos > 1)
                diffy2Pos = 1;
            if(diffy2Pos < 0)
                diffy2Pos = 0;
            diffy1.setPosition(diffy1Pos);
            diffy2.setPosition(diffy2Pos);
            updateTelemetry();
        }
    }

    public void updateTelemetry(){
        telemetry.addData("diffyServo1 position", diffy1Pos);
        telemetry.addData("diffyServo1 position", diffy2Pos);
        telemetry.update();
    }
}
