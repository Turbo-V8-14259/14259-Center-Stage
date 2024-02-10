package org.firstinspires.ftc.teamcode.opmode.tests.motorTests.eachMotorIndividual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Wrist;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

@TeleOp(name = "DEPOSITING END AFFECTOR")
public class EndAffector extends LinearOpMode {
    boolean timeToggle = true;
    double TimeStamp = 0;
    ElapsedTime timer = new ElapsedTime();
    int scoringState = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        Servo arm = hardwareMap.get(Servo.class, "arm");
//        Wrist wrist1 = new Wrist(new ServoMotorBetter(wrist));
//        DepoArm arm1 = new DepoArm(new ServoMotorBetter(arm), new ServoMotorBetter(hardwareMap.get(Servo.class, "fake")));
        stickyGamepad gamepadOne = new stickyGamepad(gamepad1);
//        wrist1.setState(Wrist.WristState.INITIALIZE);
//        arm1.setState(DepoArm.DepoArmState.INITIALIZE);
//        arm1.update();
//        wrist1.update();
//        wrist.setPosition(0);
//        arm.setPosition(0);
        waitForStart();
        while(opModeIsActive()){
            if(gamepadOne.dpad_up){
                wrist.setPosition(wrist.getPosition()+.01);
            }else if(gamepadOne.dpad_down){
                wrist.setPosition(wrist.getPosition()-.01);
            }
            if(gamepadOne.dpad_left){
                arm.setPosition(arm.getPosition()+.01);
            }else if(gamepadOne.dpad_right){
                arm.setPosition(arm.getPosition()-0.01);
            }
//            if(gamepadOne.a){
//                scoringState ++;
//            }
//            wrist1.setState(Wrist.WristState.Manual);
//            arm1.setState(DepoArm.DepoArmState.Manual);
//            if(gamepadOne.dpad_up){
//                wrist1.setManualPosition(wrist1.manualPosition+.05);
//            }else if(gamepadOne.dpad_down){
//                wrist1.setManualPosition(wrist1.manualPosition-.05);
//            }
//            if(gamepadOne.dpad_left){
//                arm1.setManualPosition(arm1.manualPosition+.05);
//            }else if(gamepadOne.dpad_right){
//                arm1.setManualPosition(arm1.manualPosition-.05);
//            }
//            if(scoringState == 1){
//                arm1.setState(DepoArm.DepoArmState.INTERMEDIATE);
//                if(timeToggle){//timeToggle starts at true by default
//                    TimeStamp = timer.milliseconds();
//                    timeToggle = false;
//                }
//                if(timer.milliseconds() > TimeStamp + 500){
//                    scoringState=2;
//                    timeToggle = true;
//                }
//            }else if(scoringState ==2){
//                wrist1.setState(Wrist.WristState.INTERMEDIATE);
//                if(timeToggle){//timeToggle starts at true by default
//                    TimeStamp = timer.milliseconds();
//                    timeToggle = false;
//                }
//                if(timer.milliseconds() > TimeStamp + 500){
//                    scoringState=3;
//                    timeToggle = true;
//                }
//            }else if(scoringState == 3){
//                arm1.setState(DepoArm.DepoArmState.LT_SCORE);
//                wrist1.setState(Wrist.WristState.LT_SCORE);
//            }else if(scoringState == 4){
//                arm1.setState(DepoArm.DepoArmState.INTERMEDIATE);
//                wrist1.setState(Wrist.WristState.INTERMEDIATE);
//                if(timeToggle){//timeToggle starts at true by default
//                    TimeStamp = timer.milliseconds();
//                    timeToggle = false;
//                }
//                if(timer.milliseconds() > TimeStamp + 500){
//                    scoringState=5;
//                    timeToggle = true;
//                }
//            }else if(scoringState == 5){
//                wrist1.setState(Wrist.WristState.ABOVE_TRANSFER);
//                if(timeToggle){//timeToggle starts at true by default
//                    TimeStamp = timer.milliseconds();
//                    timeToggle = false;
//                }
//                if(timer.milliseconds() > TimeStamp + 1000){
//                    scoringState=6;
//                    timeToggle = true;
//                }
//            }else if(scoringState == 6){
//                arm1.setState(DepoArm.DepoArmState.TRANSFER);
////                wrist1.setState(Wrist.WristState.TRANSFER); when new guide gets added
//                if(timeToggle){//timeToggle starts at true by default
//                    TimeStamp = timer.milliseconds();
//                    timeToggle = false;
//                }
//                if(timer.milliseconds() > TimeStamp + 500){
//                    scoringState=7;
//                    timeToggle = true;
//                }
//            }else if(scoringState == 7){
//                wrist1.setState(Wrist.WristState.TRANSFER);
//            }else if(scoringState == 8){
//                scoringState =1;
//            }
            gamepadOne.update();
//            arm1.update();
//            wrist1.update();
            telemetry.addData("wrist",wrist.getPosition());
            telemetry.addData("arm",arm.getPosition());
            telemetry.update();
        }
    }
}
