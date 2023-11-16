package org.firstinspires.ftc.teamcode.opmode.tests.motorTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;

@TeleOp
@Config
@Disabled

public class slidesOpmode extends LinearOpMode {
    DepoSlides slides;
    stickyGamepad gamepada;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "rightSlides")));
        slides.passive = false;
        slides.pidRunning = true;
        slides.manualMode = false;
        gamepada = new stickyGamepad(gamepad1);
        waitForStart();
        while(opModeIsActive()){
            if(gamepada.a){
                slides.setState(DepoSlides.DepositState.UP);
            }else if(gamepada.b){
                slides.setState(DepoSlides.DepositState.DOWN);
            }
            if(gamepada.dpad_up){
                slides.setState(DepoSlides.DepositState.INTERMEDIATE_INCRIMENT);
                slides.setState(DepoSlides.DepositState.INCREMENT_UP);
            }else if(gamepada.dpad_down){
                slides.setState(DepoSlides.DepositState.INCREMENT_DOWN);
            }
            telemetry.addData("Slides encoder position ", slides.leftMotor.getCurrentPositionRAW());
            telemetry.addData("slides inch ", slides.getCurrentInches());
            telemetry.addData("isAtTarg?: ", slides.isAtTarget);
            telemetry.addData("target inches", slides.getTargetInches());
            updateAll();
        }
    }
    public void updateAll(){
        slides.update();
        gamepada.update();
        telemetry.update();
    }
}
