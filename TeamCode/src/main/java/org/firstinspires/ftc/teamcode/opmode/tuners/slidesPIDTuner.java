package org.firstinspires.ftc.teamcode.opmode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Pitch;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

@Config
@TeleOp

public class slidesPIDTuner extends LinearOpMode {
    public static double setTarget = 0;
    Pitch pitch;
    LM1Turret turret;
    DepoArm arm;

    DepoSlides slides;
    stickyGamepad gamepadOne;
    @Override
    public void runOpMode() throws InterruptedException {
        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "rightSlides")));
        //initalize here
        gamepadOne = new stickyGamepad(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pitch = new Pitch(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "Pitch")));
        turret = new LM1Turret(new ServoMotorBetter(hardwareMap.get(Servo.class, "turret")));
        arm = new DepoArm(new ServoMotorBetter(hardwareMap.get(Servo.class, "arm")), new ServoMotorBetter(hardwareMap.get(Servo.class, "fake")));
        waitForStart();

        while(opModeIsActive()){

            arm.setState(DepoArm.DepoArmState.INTERMEDIATE);
            turret.setState(LM1Turret.TurretState.INITIALIZE);
            slides.manualMode = false;
            if (gamepad1.a){
                slides.target = -0.3; //always make this negative to go forard
            }else{
                slides.target = 0;
            }

            telemetry.addData("slides position", slides.getCurrentPosition());
            telemetry.addData("target", setTarget);

            //update here
            //add data here
            telemetry.update();
            pitch.update();
            arm.update();
            turret.update();
            slides.update();
        }
    }
}
