//package org.firstinspires.ftc.teamcode.opmode.tests;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
//import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
//import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
//import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
//import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;
//
//
//@TeleOp
//public class Scoring extends LinearOpMode {
//    SampleMecanumDrive drive;
//    Intake intake;
//
//    LM1Turret turret;
//    DepoArm arm;
//    stickyGamepad gamepadOne;
//    ElapsedTime timer;
//
//    double a = 0;
//
//    double TimeStamp = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        drive = new SampleMecanumDrive(hardwareMap);
//        intake = new Intake(hardwareMap.get(DcMotorEx.class, "Intake"), new ServoMotorBetter(hardwareMap.get(Servo.class, "intakeArm")));
//        intake.setState(Intake.IntakeState.INITIALIZE);
//        intake.update();
//        turret = new LM1Turret(new ServoMotorBetter(hardwareMap.get(Servo.class, "turret")));
//        arm = new DepoArm(new ServoMotorBetter(hardwareMap.get(Servo.class, "arm")), new ServoMotorBetter(hardwareMap.get(Servo.class, "fake")));
//        gamepadOne = new stickyGamepad(gamepad1);
//        timer = new ElapsedTime();
//        waitForStart();
//        while(opModeIsActive()){
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x,
//                            gamepad1.right_stick_x
//                    )
//            );
//
//            if(gamepadOne.a){
//                a++;
//            }
//
//            if(a==0){
//                arm.setState(DepoArm.DepoArmState.TRANSFER);
//            }else if(a==1){
//                turret.setState(LM1Turret.TurretState.INITIALIZE);
//                arm.setState(DepoArm.DepoArmState.INTERMEDIATE);
//            }else if(a==2){
//                turret.setState(LM1Turret.TurretState.SCORE);
//            }else if(a==3){
//                arm.setState(DepoArm.DepoArmState.SCORE);
//            }else if(a==4){
//                arm.setState(DepoArm.DepoArmState.INTERMEDIATE);
//                turret.setState(LM1Turret.TurretState.INITIALIZE);
//
//            }
//            if(a > 4){
//                a = 0;
//            }
//
//            if(gamepadOne.dpad_up){
//                intake.setState(Intake.IntakeState.INCRIMENT_UP);
//            }else if(gamepadOne.dpad_down) {
//                intake.setState(Intake.IntakeState.INCRIMENT_DOWN);
//            }
//            intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
//
//            telemetry.addData("turret", turret.getState());
//            telemetry.addData("arm", arm.getState());
//
//            turret.update();
//            arm.update();
//            gamepadOne.update();
//            telemetry.update();
//            drive.update();
//            intake.update();
//        }
//
//    }
//}
package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Pitch;
import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

import java.sql.Time;


@TeleOp
public class Scoring extends LinearOpMode {
    SampleMecanumDrive drive;
    Intake intake;

    LM1Turret turret;
    DepoArm arm;
    stickyGamepad gamepadOne;
    stickyGamepad gamepadTwo;
    ElapsedTime timer = new ElapsedTime();

    Pitch pitch;

    DepoSlides slides;

    int scoringState = 0;

    double TimeStamp = 0;

    boolean timeToggle = true;

    double height = 1;
    // why the fuck is this a double ???

    public int level = 0;  //6 levels
    public int extension = 0; //6 levels
    public boolean autoLockMode = false;
    public boolean autoIntake = false;
    @Override
    public void runOpMode() throws InterruptedException {
        pitch = new Pitch(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "Pitch")));
        pitch.manualMode = false;

        drive = new SampleMecanumDrive(hardwareMap);

        intake = new Intake(hardwareMap.get(DcMotorEx.class, "Intake"), new ServoMotorBetter(hardwareMap.get(Servo.class, "intakeArm")));
        intake.setState(Intake.IntakeState.INITIALIZE);
        intake.update();

        turret = new LM1Turret(new ServoMotorBetter(hardwareMap.get(Servo.class, "turret")));

        arm = new DepoArm(new ServoMotorBetter(hardwareMap.get(Servo.class, "arm")), new ServoMotorBetter(hardwareMap.get(Servo.class, "fake")));

        gamepadOne = new stickyGamepad(gamepad1);
        gamepadTwo = new stickyGamepad(gamepad2);

        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "rightSlides")));

        slides.passive = false;
        slides.pidRunning = true;
        slides.manualMode = false;

        waitForStart();
        while(opModeIsActive()){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            gamepad1.right_stick_x * 0.7
                    )
            );
            if(gamepadOne.dpad_up){
                level++;
            }else if(gamepadOne.dpad_down) {
                level--;
            }
            if(level > 5) level = 5;
            else if(level < 0) level = 0;
            if(gamepadOne.a){
                scoringState++;
            }
            if(gamepadOne.dpad_right){
                extension++;
            }else if(gamepadOne.dpad_left) {
                extension--;
            }
            if(extension > 5) level = 5;
            else if(extension < 0) level = 0;
            if(gamepadOne.right_bumper){
                scoringState++;
            }
            if(gamepadOne.left_bumper){
                autoIntake = !autoIntake;
            }
            updateVariable();
            scoreStateMachine();

            if(autoIntake)intake.setPower(1);
            else intake.setPower((gamepad1.left_trigger - gamepad1.right_trigger)*1);

            telemetry.addData("turret", turret.getState());
            telemetry.addData("arm", arm.getState());
            telemetry.addData("robot angle", drive.getPoseEstimate().getHeading());
            telemetry.addData("height", height); //2 is high, 1 is norm
            telemetry.addData("scoring State", scoringState); //yuh
            telemetry.addData("level", level);
            telemetry.addData("extension", extension);
            telemetry.addData("pitch", pitch.getState());
            telemetry.addData("slides", slides.getState());
            telemetry.addData("slidesTarget", slides.getTargetInches());
//            drive.setPoseEstimate(new Pose2d(0,0,0)); sets the robot to 0,0,0 (last one is the heading)

            turret.update();
            arm.update();
            gamepadOne.update();
            telemetry.update();
            drive.update();
            intake.update();
            pitch.update();
            slides.update();
        }



    }
    public void updateGamepadOne(){

    }
    public void scoreStateMachine(){
        switch (scoringState){
            case 0: //Init
                slides.setState(DepoSlides.DepositState.DOWN);
                pitch.setState(Pitch.PitchState.INITIALIZE);
                arm.setState(DepoArm.DepoArmState.TRANSFER);
                break;
            case 1: //arm / pitch / slides up
                pitch.setState(Pitch.PitchState.SCOREATLEVEL);
                arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);
                slides.setState(DepoSlides.DepositState.CALCULATED_UP);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds()> TimeStamp + 500){
                    scoringState=2;
                    timeToggle = true;
                }
                break;
            case 2: // turret move
                if(autoLockMode) turret.setState(LM1Turret.TurretState.AUTOLOCK);
                else turret.setState(LM1Turret.TurretState.SCORE);
                break;
            case 3:
                arm.setState(DepoArm.DepoArmState.SCORE);
                break;
            case 4: // raising ARM
                arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);
                if(timeToggle){
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds()>TimeStamp + 100){
                    scoringState=5;
                    timeToggle=true;
                }
                break;
            case 5: //resetting turret
                turret.setState(LM1Turret.TurretState.INITIALIZE);
                pitch.setState(Pitch.PitchState.INITIALIZE);
                slides.setState(DepoSlides.DepositState.DOWN);
                if(timeToggle){
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds()>TimeStamp + 500){
                    scoringState=6;
                    timeToggle=true;
                }
                break;
            default:
                scoringState = 0;
                break;

        }
    }
    public void updateVariable(){
        pitch.level = this.level;
        arm.level = this.level;
        slides.level = this.level;
        slides.extension = this.extension;
    }
}
