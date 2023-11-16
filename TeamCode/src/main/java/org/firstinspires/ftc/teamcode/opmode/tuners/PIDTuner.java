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
@Disabled

public class PIDTuner extends LinearOpMode {
    public static double setTarget = 0;
    Pitch pitch;
    LM1Turret turret;
    DepoArm arm;

    @Override
    public void runOpMode() throws InterruptedException {
        //initalize here
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pitch = new Pitch(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "Pitch")));
        turret = new LM1Turret(new ServoMotorBetter(hardwareMap.get(Servo.class, "turret")));
        arm = new DepoArm(new ServoMotorBetter(hardwareMap.get(Servo.class, "arm")), new ServoMotorBetter(hardwareMap.get(Servo.class, "fake")));
        waitForStart();

        while(opModeIsActive()){
            arm.setState(DepoArm.DepoArmState.INTERMEDIATE);
            turret.setState(LM1Turret.TurretState.INITIALIZE);
            pitch.manualMode = false;
            pitch.target = setTarget;

            telemetry.addData("pitch position", pitch.getCurrentPosition());
            telemetry.addData("target", setTarget);
            //update here
            //add data here
            telemetry.update();
            pitch.update();
            arm.update();
            turret.update();
        }
    }
}
