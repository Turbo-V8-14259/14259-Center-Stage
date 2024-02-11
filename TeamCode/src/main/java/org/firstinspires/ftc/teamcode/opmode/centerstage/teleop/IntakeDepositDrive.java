package org.firstinspires.ftc.teamcode.opmode.centerstage.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Claw;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Pitch;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Wrist;
import org.firstinspires.ftc.teamcode.hardware.Intake.LTIntake;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

@TeleOp(name = "!R for REDEMPTION!")
public class IntakeDepositDrive extends LinearOpMode {
    DepoSlides slides;
    boolean timeToggle = true;
    double TimeStamp = 0;
    ElapsedTime timer = new ElapsedTime();
    int scoringState = 5;
    Pitch pitch;
<<<<<<< HEAD
    int climbSafe = 0;

    boolean intakingDriveMove = true;
=======
>>>>>>> b139d3a909942ccc59ad36263f929290f5bc9f63
    @Override
    public void runOpMode() throws InterruptedException {
        pitch = new Pitch(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "Pitch")));
        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class,"leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class,"rightSlides")));
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        Claw claw = new Claw(new ServoMotorBetter(hardwareMap.get(Servo.class, "claw")));
        Servo intakeArm1 = hardwareMap.get(Servo.class, "intakeArm");
        Servo intakeArm2 = hardwareMap.get(Servo.class, "intakeArm2");
        LTIntake intake = new LTIntake(intakeMotor, new ServoMotorBetter(intakeArm2), new ServoMotorBetter(intakeArm1));
        stickyGamepad gamepad = new stickyGamepad(gamepad1);
        stickyGamepad Gamepad2 = new stickyGamepad(gamepad2);

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
            if(Gamepad2.left_bumper){
                slides.setSpaceInInches(slides.getSpaceInInches()+1);
                if(slides.getSpaceInInches()+1 >= slides.inches.length){
                    slides.setSpaceInInches(0);
                }
            }
            if(Gamepad2.right_bumper){
//                arm1.setLevel(pitch.getAngle());
                wrist1.setLevel(pitch.getAngle());

                if(!(pitch.getAngle()+1 >= pitch.angles.length)){
                    pitch.setAngle(pitch.getAngle() + 1);
                }else {
                    pitch.setAngle(0);
                }
            }
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
            intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            gamepadOne.update();
            arm1.update();
            wrist1.update();
            intake.update();
            claw.update();
            slides.update();
            pitch.update();
            Gamepad2.update();
            if(scoringState == 1){
                arm1.setState(DepoArm.DepoArmState.INTERMEDIATE);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 150){
                    scoringState=2;
                    timeToggle = true;
                }
            }else if(scoringState ==2){
                wrist1.setState(Wrist.WristState.INTERMEDIATE);
                pitch.setState(Pitch.PitchState.CALCULATED_UP);
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
                slides.setState(DepoSlides.DepositState.CALCULATED_UP);
            }else if(scoringState == 4){
                claw.setState(Claw.ClawState.UNLATCHED);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 500){
                    scoringState=5;
                    timeToggle = true;
                }
            }else if(scoringState == 5){
                pitch.setState(Pitch.PitchState.INITIALIZE);
                slides.setState(DepoSlides.DepositState.DOWN);
                arm1.setState(DepoArm.DepoArmState.INTERMEDIATE);
                wrist1.setState(Wrist.WristState.ABOVE_TRANSFER);
                if(pitch.getAngle()==0){
                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + 500){
                        scoringState=6;
                        timeToggle = true;
                    }
                }else if(Math.abs(pitch.getCurrentPosition() - pitch.target) < .05){
                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + 500){
                        scoringState=6;
                        timeToggle = true;
                    }
                }


            }else if(scoringState == 6){
                arm1.setState(DepoArm.DepoArmState.ABOVE_TRANSFER);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 500){
                    scoringState=7;
                    timeToggle = true;
                }
            }else if(scoringState == 7){
                arm1.setState(DepoArm.DepoArmState.TRANSFER);
                wrist1.setState(Wrist.WristState.TRANSFER); //when new guide gets added
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 500){
                    scoringState=8;
                    timeToggle = true;
                }
            }else if(scoringState == 8){
                wrist1.setState(Wrist.WristState.TRANSFER);
                slides.setState(DepoSlides.DepositState.OVER_IN);
            }else if(scoringState == 9){
                claw.setState(Claw.ClawState.LATCHED);
<<<<<<< HEAD
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 300){
                    scoringState = 1;
                    timeToggle = true;
                }
            }else if(scoringState == 10){
                arm1.setState(DepoArm.DepoArmState.INTERMEDIATE);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 100){
                    scoringState=11;
                    timeToggle = true;
                }
            } else if (scoringState == 11) {
                arm1.setState(DepoArm.DepoArmState.CLIMB);
                wrist1.setState(Wrist.WristState.LT_SCORE);
                pitch.setState(Pitch.PitchState.CLIMB);
            }else if(scoringState == 12){
                intake.setState(LTIntake.IntakeState.INITIALIZE);
                pitch.manualMode = true;
                pitch.setPowerManual(-gamepad2.left_stick_y);
=======
                scoringState = 1;
>>>>>>> b139d3a909942ccc59ad36263f929290f5bc9f63
            }
            if(gamepad.dpad_up){
                intake.setState(LTIntake.IntakeState.INITIALIZE);
            }else if(gamepad.dpad_down) {
                intake.setState(LTIntake.IntakeState.INTAKE_TELE);
            }
<<<<<<< HEAD
            telemetry.addData("pitch level ",pitch.getAngle());
            telemetry.addData("slides level ", slides.getSpaceInInches());
            telemetry.addData("intake forward? ", intakingDriveMove);
            telemetry.addData("scoring state", scoringState);
            telemetry.update();
=======
>>>>>>> b139d3a909942ccc59ad36263f929290f5bc9f63
        }
    }
}
