package org.firstinspires.ftc.teamcode.opmode.tests.servoTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;


@TeleOp
public class turretManualMove extends LinearOpMode {
    LM1Turret turret;
    stickyGamepad gamepadOne;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepadOne = new stickyGamepad(gamepad1);
        turret = new LM1Turret(new ServoMotorBetter(hardwareMap.get(Servo.class, "turret")));
        waitForStart();
        while(opModeIsActive()){
            turret.setState(LM1Turret.TurretState.RUNTOPOSITION);
            if(gamepadOne.left_bumper){
                turret.manualPosition +=.05;
            }else if(gamepadOne.right_bumper){
                turret.manualPosition -=.05;
            }
            turret.update();
            telemetry.addData("turret position", turret.getPosition());
            telemetry.update();
            gamepadOne.update();
        }
    }
}
