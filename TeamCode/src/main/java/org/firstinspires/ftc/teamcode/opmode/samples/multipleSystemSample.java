package org.firstinspires.ftc.teamcode.opmode.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoTurret;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;

@TeleOp
@Config
public class multipleSystemSample extends LinearOpMode {
    DepoSlides slides;
    DepoTurret turret;
    stickyGamepad gamepada;


    public static double bothTarget = 0;
    double error;

    @Override
    public void runOpMode() throws InterruptedException {

        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "rightSlides")));
        turret = new DepoTurret(hardwareMap.get(CRServo.class, "axon"), hardwareMap.get(AnalogInput.class, "input"));
        gamepada = new stickyGamepad(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()){
            if(gamepada.a){
                bothTarget = 1;
            }else if(gamepada.b){
                bothTarget = 3;
            }else if(gamepada.x){
                bothTarget = 0;
            }
            if(bothTarget==1){
                slides.setState(DepoSlides.DepositState.UP);
            }else if(bothTarget == 2){
                slides.setState(DepoSlides.DepositState.MIDDLE);
            }else if(bothTarget==3){
                slides.setState(DepoSlides.DepositState.DOWN);
            }else if(bothTarget ==0){
                slides.setState(DepoSlides.DepositState.STOPPED);
            }

            error = Math.abs(slides.getCurrentPosition() - slides.target);
            telemetry.addData("turret target", turret.target);

            //basically acts at the end of the action of the error; in this case slides
            if(error < 0.1){
                turret.target = bothTarget * 100;
            }

            telemetry.addData("slides state", slides.getState());
            telemetry.addData("error",error);
            telemetry.addData("slides pos ", slides.getCurrentPosition());
            telemetry.addData("slides inch ", slides.getCurrentInches());
            telemetry.addData("target ", bothTarget);
            telemetry.addData("isAtTarg?: ", slides.isAtTarget);
            updateAll();
        }

    }
    public void updateAll(){
        slides.update();
        turret.update();
        gamepada.update();
        telemetry.update();
    }
}
