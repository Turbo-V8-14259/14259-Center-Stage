package org.firstinspires.ftc.teamcode.opmode.tests.motorTests.eachMotorIndividual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Deposit.Claw;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

@TeleOp(name = "LATCH TEST")

public class Latch extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(new ServoMotorBetter(hardwareMap.get(Servo.class, "claw")));
//        Servo claw = hardwareMap.get(Servo.class, "claw");
        stickyGamepad gamepad = new stickyGamepad(gamepad1);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad.dpad_up){
//                claw.setPosition(claw.getPosition()+0.01);
                claw.setState(Claw.ClawState.LATCHED);
            }else if(gamepad.dpad_down){
//                claw.setPosition(claw.getPosition()-0.01);
                claw.setState(Claw.ClawState.UNLATCHED);
            }
//            telemetry.addData("claw position", claw.getPosition()); //0.44 is open, 0.35 is closed
            telemetry.addData("claw state", claw.getState());
            telemetry.update();
            claw.update();
            gamepad.update();
        }
    }
}
