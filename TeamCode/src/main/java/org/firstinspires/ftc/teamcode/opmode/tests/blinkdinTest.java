//package org.firstinspires.ftc.teamcode.opmode.tests;
//
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.qualcomm.hardware.rev.RevSPARKMini;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.hardware.Sensors.Blinkdin;
//import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;
//
//
//@TeleOp
//public class blinkdinTest extends LinearOpMode {
//    Blinkdin led;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        led = new Blinkdin(hardwareMap.get(Servo.class, "led"));
//        waitForStart();
//        while(opModeIsActive()){
//            led.changePattern(gamepad1.left_trigger);
//            led.update();
//        }
//
//    }
//}
