package org.firstinspires.ftc.teamcode.opmode.tests.motorTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Claw;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Wrist;
import org.firstinspires.ftc.teamcode.hardware.Intake.LTIntake;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

@TeleOp
public class IntakeDepositDrive extends LinearOpMode {
    boolean timeToggle = true;
    double TimeStamp = 0;
    ElapsedTime timer = new ElapsedTime();
    int scoringState = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        Claw claw = new Claw(new ServoMotorBetter(hardwareMap.get(Servo.class, "claw")));
        Servo intakeArm1 = hardwareMap.get(Servo.class, "intakeArm");
        Servo intakeArm2 = hardwareMap.get(Servo.class, "intakeArm2");
        LTIntake intake = new LTIntake(intakeMotor, new ServoMotorBetter(intakeArm2), new ServoMotorBetter(intakeArm1));
        stickyGamepad gamepad = new stickyGamepad(gamepad1);
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        Servo arm = hardwareMap.get(Servo.class, "arm");
        Wrist wrist1 = new Wrist(new ServoMotorBetter(wrist));
        DepoArm arm1 = new DepoArm(new ServoMotorBetter(arm), new ServoMotorBetter(hardwareMap.get(Servo.class, "fake")));
        stickyGamepad gamepadOne = new stickyGamepad(gamepad1);
        wrist1.setState(Wrist.WristState.INITIALIZE);
        arm1.setState(DepoArm.DepoArmState.INITIALIZE);
        arm1.update();
        wrist1.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake.setState(LTIntake.IntakeState.INTAKE_TELE);
        intake.update();
        waitForStart();
        while (opModeIsActive()){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();
            if(gamepadOne.a){
                scoringState ++;
            }
            if(scoringState == 1){
                arm1.setState(DepoArm.DepoArmState.INTERMEDIATE);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 500){
                    scoringState=2;
                    timeToggle = true;
                }
            }else if(scoringState ==2){
                wrist1.setState(Wrist.WristState.INTERMEDIATE);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 500){
                    scoringState=3;
                    timeToggle = true;
                }
            }else if(scoringState == 3){
                arm1.setState(DepoArm.DepoArmState.LT_SCORE);
                wrist1.setState(Wrist.WristState.LT_SCORE);
            }else if(scoringState == 4){
                claw.setState(Claw.ClawState.UNLATCHED);
                arm1.setState(DepoArm.DepoArmState.INTERMEDIATE);
                wrist1.setState(Wrist.WristState.INTERMEDIATE);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 500){
                    scoringState=5;
                    timeToggle = true;
                }
            }else if(scoringState == 5){
                wrist1.setState(Wrist.WristState.ABOVE_TRANSFER);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 1000){
                    scoringState=6;
                    timeToggle = true;
                }
            }else if(scoringState == 6){
                arm1.setState(DepoArm.DepoArmState.TRANSFER);
//                wrist1.setState(Wrist.WristState.TRANSFER); when new guide gets added
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 500){
                    scoringState=7;
                    timeToggle = true;
                }
            }else if(scoringState == 7){
                wrist1.setState(Wrist.WristState.TRANSFER);
            }else if(scoringState == 8){
                claw.setState(Claw.ClawState.LATCHED);
                scoringState =1;
            }
            if(gamepad.dpad_up){
                intake.setState(LTIntake.IntakeState.INITIALIZE);
            }else if(gamepad.dpad_down) {
                intake.setState(LTIntake.IntakeState.INTAKE_TELE);
            }
            intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            gamepadOne.update();
            arm1.update();
            wrist1.update();
            intake.update();
            claw.update();
        }
    }
}
