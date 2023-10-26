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
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;

@TeleOp
@Config
public class multipleSystemTest extends LinearOpMode {

    public static double bothTarget = 0;
    double error;

    @Override
    public void runOpMode() throws InterruptedException {

        DepoSlides slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "rightSlides")));
        DepoTurret turret = new DepoTurret(hardwareMap.get(CRServo.class, "axon"), hardwareMap.get(AnalogInput.class, "input"));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()){

            error = Math.abs(slides.getCurrentPosition()-bothTarget);

            slides.target = bothTarget;

            //basically acts at the end of the action of the error; in this case slides
            if(error < .01){
                turret.target = bothTarget * 100;
            }

            slides.update();
            turret.update();

            telemetry.addData("error",error);
            telemetry.update();
        }

    }
}