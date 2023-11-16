package org.firstinspires.ftc.teamcode.opmode.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled

public class multipleSystemSample extends LinearOpMode {
    DepoSlides slides;
    DepoTurret turret;
    stickyGamepad gamepada;

    @Override
    public void runOpMode() throws InterruptedException {

        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "rightSlides")));
        turret = new DepoTurret(hardwareMap.get(CRServo.class, "axon"), hardwareMap.get(AnalogInput.class, "input"));
        gamepada = new stickyGamepad(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()){
            if(gamepada.a){
                slides.setState(DepoSlides.DepositState.UP);
            }else if(gamepada.b){
                slides.setState(DepoSlides.DepositState.DOWN);
            }else if(gamepada.x){
                slides.setState(DepoSlides.DepositState.STOPPED);
            }

            updateAll(); //order of operations bruv

            //basically acts at the end of the action of the error; in this case slides
            if(slides.getState()==DepoSlides.DepositState.UP&&slides.isAtTarget){
                turret.setState(DepoTurret.TurretState.TELE_SCORING);
            }else{
                turret.setState(DepoTurret.TurretState.TRANSFER);
            }

            telemetry.addData("turret target", turret.target);
            telemetry.addData("slides inch ", slides.getCurrentInches());
            telemetry.addData("isAtTarg?: ", slides.isAtTarget);
            telemetry.addData("slides state", slides.getState());
            telemetry.addData("turret state", turret.getState());
        }

    }
    public void updateAll(){
        slides.update();
        turret.update();
        gamepada.update();
        telemetry.update();
    }
}
