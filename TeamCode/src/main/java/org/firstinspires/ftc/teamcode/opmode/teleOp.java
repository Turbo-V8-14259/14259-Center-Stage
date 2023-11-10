package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoTurret;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Pitch;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;

@TeleOp
@Config
public class teleOp extends LinearOpMode {
    Robot robot;
    DepoSlides slides;
    DepoTurret turret;
    DepoArm depoArm;
    Pitch pitch;
    stickyGamepad gamepadOne;
    int slidesController = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        pitch = new Pitch(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "Pitch")));
        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "rightSlides")));
        turret = new DepoTurret(hardwareMap.get(CRServo.class, "axon"), hardwareMap.get(AnalogInput.class, "input"));
        gamepadOne = new stickyGamepad(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        while(opModeIsActive()){

            if(gamepadOne.dpad_up){
                slidesController = 1;
            }else if(gamepadOne.dpad_down){
                slidesController = 2;
            }


            if(slidesController == 1){
                slides.setState(DepoSlides.DepositState.INCREMENT_UP);
            }else if(slidesController == 2){
                slides.setState(DepoSlides.DepositState.INCREMENT_DOWN);
            }

            updateAll();
        }
    }
    public void updateAll(){
        slides.update();
        turret.update();
        pitch.update();
        gamepadOne.update();
        telemetry.update();
    }
}
