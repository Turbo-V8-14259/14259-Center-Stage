package org.firstinspires.ftc.teamcode.opmode.centerstage.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
import org.firstinspires.ftc.teamcode.hardware.Sensors.Blinkdin;
import org.firstinspires.ftc.teamcode.hardware.Sensors.PixelSensor;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

@TeleOp(name = "Houston Bound")
public class RegionalsTeleOP extends LinearOpMode {
    DepoSlides slides;
    boolean timeToggle = true;
    double TimeStamp = 0;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime pixelTimer = new ElapsedTime();
    int scoringState = 5;
    Pitch pitch;
    int climbSafe = 0;
    int ledState = 0;
    Blinkdin led;
    boolean intakingDriveMove = true;
    PixelSensor sensor1;
    PixelSensor sensor2;
    DcMotorEx intakeMotor;;
    Claw claw;
    Servo intakeArm1;
    Servo intakeArm2;
    LTIntake intake;
    Wrist wrist1;
    DepoArm arm1;
    SampleMecanumDrive drive;
    stickyGamepad gamepadOne;
    stickyGamepad Gamepad2;
    stickyGamepad gamepad;

    boolean colorMode = true;
    @Override
    public void runOpMode() throws InterruptedException {
        pitch = new Pitch(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "Pitch")));
        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class,"leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class,"rightSlides")));
        led = new Blinkdin(hardwareMap.get(RevBlinkinLedDriver.class, "led"));
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        claw = new Claw(new ServoMotorBetter(hardwareMap.get(Servo.class, "claw")));
        sensor1 = new PixelSensor(hardwareMap.get(ColorSensor.class, "Color1"));
        sensor2 = new PixelSensor(hardwareMap.get(ColorSensor.class, "Color2"));
        intakeArm1 = hardwareMap.get(Servo.class, "intakeArm");
        intakeArm2 = hardwareMap.get(Servo.class, "intakeArm2");
        intake = new LTIntake(intakeMotor, new ServoMotorBetter(intakeArm2), new ServoMotorBetter(intakeArm1));
        gamepadOne = new stickyGamepad(gamepad1);
        Gamepad2 = new stickyGamepad(gamepad2);
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        Servo arm = hardwareMap.get(Servo.class, "arm");
        wrist1 = new Wrist(new ServoMotorBetter(wrist));
        arm1 = new DepoArm(new ServoMotorBetter(arm), new ServoMotorBetter(hardwareMap.get(Servo.class, "fake")));
        wrist1.setState(Wrist.WristState.INITIALIZE);
        arm1.setState(DepoArm.DepoArmState.INITIALIZE);
        arm1.update();
        wrist1.update();
        drive = new SampleMecanumDrive(hardwareMap);
        intake.setState(LTIntake.IntakeState.INTAKE_TELE);
        intake.update();
        waitForStart();
        while (opModeIsActive()){
            intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
            if(intakingDriveMove){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                gamepad1.right_stick_x
                        )
                );
            }else{
                drive.setWeightedDrivePower(
                        new Pose2d(
                                gamepad1.left_stick_y,
                                gamepad1.left_stick_x,
                                gamepad1.right_stick_x
                        )
                );
            }

            if(ledState == 0){
                led.changePattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }else if(ledState == 1){
                led.changePattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            }else if(ledState == 2) {
                led.changePattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }else if(ledState == 3) {
                led.changePattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }else if(ledState >3){
                ledState = 0;
            }
            drive.update();
            arm1.update();
            wrist1.update();
            intake.update();
            claw.update();
            slides.update();
            pitch.update();
            led.update();
            if(colorMode){
                this.updateColorFSM();
            }else{
                this.updateNonColorFSM();
            }
            this.updateGamepads();
            this.updateTelemetry();
        }
    }


    public void updateTelemetry(){
        telemetry.addData("pitch level ",pitch.getAngle());
        telemetry.addData("slides level ", slides.getSpaceInInches());
        telemetry.addData("intake forward? ", intakingDriveMove);
        telemetry.addData("scoring state", scoringState);
        telemetry.update();
    }




    public void updateGamepads(){
        if(Gamepad2.x){
            climbSafe++;
        }
        if(climbSafe == 3){
            scoringState = 10;
        }else if(climbSafe == 5){
            scoringState = 12;
        }
        if(gamepadOne.left_bumper){
            intakingDriveMove = true;
        }else if(gamepadOne.right_bumper){
            intakingDriveMove = false;
        }
        if(gamepadOne.a || Gamepad2.a){
            scoringState ++;
        }
        if(Gamepad2.y){
            slides.setSpaceInInches(0);
            pitch.setAngle(0);
        }
        if(Gamepad2.left_bumper){
            if(!(slides.getSpaceInInches()+1 >= slides.inches.length)){
                slides.setSpaceInInches(slides.getSpaceInInches()+1);
            }else{
                slides.setSpaceInInches(0);
            }
        }
        if(Gamepad2.right_bumper){
            wrist1.setLevel(pitch.getAngle());

            if(!(pitch.getAngle()+1 >= 5)){
                pitch.setAngle(pitch.getAngle() + 1);
            }else {
                pitch.setAngle(0);
            }
        }
        if(Gamepad2.b){
            ledState++;
        }
        gamepadOne.update();
        Gamepad2.update();
    }

    public void updateColorFSM(){
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
                pitch.setState(Pitch.PitchState.SLIGHT_UP);
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
            wrist1.setState(Wrist.WristState.TRANSFER);
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
            if(!sensor1.getColor().equals("NOTHING")&&!sensor2.getColor().equals("NOTHING")){
                claw.setState(Claw.ClawState.LATCHED);
            }else{
                claw.setState(Claw.ClawState.UNLATCHED);
            }
        }else if(scoringState == 9){
            claw.setState(Claw.ClawState.LATCHED);
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
        }
        if(gamepad.dpad_up){
            intake.setState(LTIntake.IntakeState.INITIALIZE);
        }else if(gamepad.dpad_down) {
            intake.setState(LTIntake.IntakeState.INTAKE_TELE);
        }
    }
    public void updateNonColorFSM(){
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
                pitch.setState(Pitch.PitchState.SLIGHT_UP);
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
            wrist1.setState(Wrist.WristState.TRANSFER);
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
        }
        if(gamepad.dpad_up){
            intake.setState(LTIntake.IntakeState.INITIALIZE);
        }else if(gamepad.dpad_down) {
            intake.setState(LTIntake.IntakeState.INTAKE_TELE);
        }
    }



}