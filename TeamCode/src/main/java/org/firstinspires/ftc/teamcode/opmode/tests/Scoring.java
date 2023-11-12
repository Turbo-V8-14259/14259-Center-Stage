package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;


@TeleOp
public class Scoring extends LinearOpMode {
    SampleMecanumDrive drive;
    Intake intake;

    LM1Turret turret;
    DepoArm arm;
    stickyGamepad gamepadOne;

    double a = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "Intake"), new ServoMotorBetter(hardwareMap.get(Servo.class, "intakeArm")));
        intake.setState(Intake.IntakeState.INITIALIZE);
        intake.update();
        turret = new LM1Turret(new ServoMotorBetter(hardwareMap.get(Servo.class, "turret")));
        arm = new DepoArm(new ServoMotorBetter(hardwareMap.get(Servo.class, "arm")), new ServoMotorBetter(hardwareMap.get(Servo.class, "fake")));
        gamepadOne = new stickyGamepad(gamepad1);
        waitForStart();
        while(opModeIsActive()){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if(gamepadOne.a){
                a++;
            }
            if(a==0){
                arm.setState(DepoArm.DepoArmState.INITIALIZE);
            }else if(a==1){
                turret.setState(LM1Turret.TurretState.INITIALIZE);
                arm.setState(DepoArm.DepoArmState.INTERMEDIATE);
            }else if(a==2){
                turret.setState(LM1Turret.TurretState.SCORE);
            }else if(a==3){
                arm.setState(DepoArm.DepoArmState.SCORE);
            }else if(a==4){
                turret.setState(LM1Turret.TurretState.INITIALIZE);
            }

            if(a>4){
                a = 0;
            }

            if(gamepadOne.dpad_up){
                intake.setState(Intake.IntakeState.INCRIMENT_UP);
            }else if(gamepadOne.dpad_down) {
                intake.setState(Intake.IntakeState.INCRIMENT_DOWN);
            }
            intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

            telemetry.addData("turret", turret.getState());
            telemetry.addData("arm", arm.getState());

            turret.update();
            arm.update();
            gamepadOne.update();
            telemetry.update();
            drive.update();
            intake.update();
        }

    }
}
