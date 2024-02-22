package org.firstinspires.ftc.teamcode.opmode.tests.motorTests.eachMotorIndividual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;


@TeleOp(name = "Drone tuner")
public class droneTest extends LinearOpMode {

    Servo drone;
    stickyGamepad gamepadOne;
    @Override
    public void runOpMode() throws InterruptedException {
        drone = hardwareMap.get(Servo.class, "drone");
        gamepadOne = new stickyGamepad(gamepad1);
        waitForStart();
        while(opModeIsActive()){
            if(gamepadOne.dpad_up){
                drone.setPosition(drone.getPosition()+0.01);
            }else if(gamepadOne.dpad_down){
                drone.setPosition(drone.getPosition()-0.01);
            }
            telemetry.addData("drone position", drone.getPosition());
            gamepadOne.update();
            telemetry.update();
        }
    }
}
