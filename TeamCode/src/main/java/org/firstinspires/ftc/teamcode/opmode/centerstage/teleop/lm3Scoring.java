package org.firstinspires.ftc.teamcode.opmode.centerstage.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Claw;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Pitch;
import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.hardware.Sensors.Blinkdin;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

@TeleOp(name = "LM3 Tele Op")
public class lm3Scoring extends LinearOpMode {
    SampleMecanumDrive drive;
    Intake intake;
    Claw claw;
    LM1Turret turret;
    DepoArm arm;
    stickyGamepad gamepadOne;
    stickyGamepad gamepadTwo;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime timer3 = new ElapsedTime();

    Pitch pitch;

    DepoSlides slides;
    Blinkdin led;

    int ledState = 0;

    double TimeStamp = 0;
    double TimeStamp2 = 0;
    double TimeStamp3 = 0;

    int climbSafe = 0;

    boolean timeToggle = true;
    boolean timeToggle2 = true;
    boolean timeToggle3 = true;
    double adjustedAngle = 0;
    double boardAngle = 0;

    int height = 1;
    int scoringState = -1;
    int pitchSlidesState = -1;


    public int level = 0;  //6 levels
    public int extension = 0; //6 levels
    public boolean autoLockMode = false;
    public boolean autoIntake = false;
    boolean thisToggle = true;
    @Override
    public void runOpMode() throws InterruptedException {
        claw = new Claw(new ServoMotorBetter(hardwareMap.get(Servo.class, "claw")));
        led = new Blinkdin(hardwareMap.get(RevBlinkinLedDriver.class, "led"));
        pitch = new Pitch(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "Pitch")));
        pitch.manualMode = false;

        drive = new SampleMecanumDrive(hardwareMap);

        intake = new Intake(hardwareMap.get(DcMotorEx.class, "Intake"), new ServoMotorBetter(hardwareMap.get(Servo.class, "intakeArm")));
        intake.setState(Intake.IntakeState.INTAKE_TELE);
        intake.update();

        turret = new LM1Turret(new ServoMotorBetter(hardwareMap.get(Servo.class, "turret")));

        arm = new DepoArm(new ServoMotorBetter(hardwareMap.get(Servo.class, "arm")), new ServoMotorBetter(hardwareMap.get(Servo.class, "fake")));

        gamepadOne = new stickyGamepad(gamepad1);
        gamepadTwo = new stickyGamepad(gamepad2);

        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "rightSlides")));

        slides.passive = false;
        slides.pidRunning = true;
        slides.manualMode = false;
        led.changePattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        led.update();
        waitForStart();
        while(opModeIsActive()){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x * 0.7
                    )
            );
            updateGamepadOne();
            updateGamepadTwo();
            updateVariables();
            scoringStateMachine();

            turret.update();
            arm.update();
            gamepadOne.update();
            gamepadTwo.update();
            drive.update();
            intake.update();
            pitch.update();
            slides.update();
            claw.update();

            telemetryData();
            led.update();
        }
    }

    public void telemetryData(){
        telemetry.addData("claw state", claw.getState());
        telemetry.addData("intake pos", intake.getPosition());
        telemetry.addData("color", led.getPattern());
        telemetry.addData("level", level);
        telemetry.addData("extension", extension);
        telemetry.addData("climb safe", climbSafe);
//        telemetry.addData("turret", turret.getState());
        telemetry.addData("arm", arm.getState());
        telemetry.addData("robot angle", drive.getPoseEstimate().getHeading());
//        telemetry.addData("height", height); //2 is high, 1 is norm
        telemetry.addData("scoring State", scoringState);
        telemetry.addData("pitch", pitch.getState());
//        telemetry.addData("slides", slides.getState());
        telemetry.addData("slidesTarget", slides.getCurrentPosition());
//        telemetry.addData("adjusted angle", adjustedAngle);
        telemetry.addData("auto lock Mode", autoLockMode);
        telemetry.addData("AMPS(not including drive)", slides.leftMotor.getCurrentAMPS() + slides.rightMotor.getCurrentAMPS() + intake.intakeMotor.getCurrent(CurrentUnit.AMPS) + pitch.pitchMotor.getCurrentAMPS());
        telemetry.update();
    }

    public void updateGamepadOne(){
//        if(gamepadOne.y) autoLockMode = !autoLockMode;
//        if(gamepadOne.right_stick_button) boardAngle = drive.getPoseEstimate().getHeading();

        if(gamepadOne.dpad_up) {
            level++;
            gamepad1.rumble(200);
        }
        if(gamepadOne.dpad_down) {
            level--;
            gamepad1.rumble(200);
        }

        if(level > 5) level = 5;
        if(level < 0) level = 0;

        if(gamepadOne.dpad_right) {
            extension++;
            gamepad1.rumble(100);
        }
        if(gamepadOne.dpad_left) {
            extension--;
            gamepad1.rumble(100);
        }
        if(extension > 5) extension = 5;
        if(extension < 0) extension = 0;

        if(gamepadOne.right_bumper) scoringState++;
        if(gamepadOne.left_bumper) scoringState--;
        if(gamepadOne.b) scoringState = 100; //full reset
    }
    public void scoringStateMachine(){
        switch(pitchSlidesState){
            case -1:
                slides.setState(DepoSlides.DepositState.DOWN);
                if(climbSafe!=3&&climbSafe!=4) {
                    pitch.setState(Pitch.PitchState.INITIALIZE);
                }
                break;
            case 1:
                pitch.setState(Pitch.PitchState.SCOREATLEVEL);
                break;

            case 2:
                slides.setState(DepoSlides.DepositState.CALCULATED_UP);
                break;
            case 6:
                slides.setState(DepoSlides.DepositState.DOWN);
                if(-slides.getCurrentPosition() > 0.2){
                    break;
                }
                pitch.setState(Pitch.PitchState.INITIALIZE);
                break;

            case 100:
                //looses control of the mechanisms.
                break;

        }
        switch (scoringState){
            case -1: //Init
//                slides.setState(DepoSlides.DepositState.DOWN);
                pitchSlidesState = -1;
                claw.setState(Claw.ClawState.INTAKE);
                if(climbSafe!=3&&climbSafe!=4){
//                    pitch.setState(Pitch.PitchState.INITIALIZE);
                    if(slides.isAtTarget){
                        arm.setState(DepoArm.DepoArmState.TRANSFER);
                    }
                }
                break;
            case 0:
                claw.setState(Claw.ClawState.LATCHED);
                thisToggle = true;
                break;
            case 1: //arm / pitch / slides up
//                pitch.setState(Pitch.PitchState.SCOREATLEVEL);
//                if(thisToggle){
                    pitchSlidesState = 1;
//                    thisToggle = false;
//                }
//                if(Math.abs(pitch.getCurrentPosition() - pitch.target) < .1){
//                    pitchSlidesState = 2;
//                }
                arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 1000){
                    scoringState=2;
                    timeToggle = true;
                }//change this to be position of pitch baseed
                break;
            case 2: // turret move
//                slides.setState(DepoSlides.DepositState.CALCULATED_UP);
                pitchSlidesState = 2;
                if(autoLockMode) {
                    //turret.robotAngle =
                    turret.setState(LM1Turret.TurretState.AUTOLOCK);
                }
                else turret.setState(LM1Turret.TurretState.SCORE);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                    timeToggle2 = false;
                }
                if(timer.milliseconds()> TimeStamp + 400){
                    arm.setState(DepoArm.DepoArmState.SCORE);
                    timeToggle = true;
                    timeToggle2 = true;
                }
                if(timeToggle2){//timeToggle starts at true by default
                    TimeStamp2 = timer2.milliseconds();
                    timeToggle2 = false;
                }
                if(timer2.milliseconds() > TimeStamp2 + 650){
                    scoringState = 3;
                    timeToggle2 = true;
                }

                break;
            case 3:
                break;
            case 4:
                claw.setState(Claw.ClawState.UNLATCHED);
                break;
            case 5: // raising ARM
                claw.setState(Claw.ClawState.LATCHED);
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
            case 6: //resetting turret
                turret.setState(LM1Turret.TurretState.INITIALIZE);
//                slides.setState(DepoSlides.DepositState.DOWN);
//                if(-slides.getCurrentPosition() > 0.2){
//                    break;
//                }
//                pitch.setState(Pitch.PitchState.INITIALIZE);
                pitchSlidesState = 6;
                if(timeToggle){
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds()>TimeStamp + 500){
                    scoringState=-1;
                    timeToggle=true;
                }

                break;
            case 100: //universal reset
                pitchSlidesState = 100;
                arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);
                if(timeToggle3){
                    TimeStamp3 = timer3.milliseconds();
                    timeToggle3 = false;
                }
                if(timer.milliseconds() > TimeStamp3 + 500){
                    scoringState = 101;
                    timeToggle3 = true;
                }
                break;
            case 101:
                pitchSlidesState = 100;
                turret.setState(LM1Turret.TurretState.INITIALIZE);
                if(timeToggle3){
                    TimeStamp3 = timer3.milliseconds();
                    timeToggle3 = false;
                }
                if(timer.milliseconds() > TimeStamp3 + 700){
                    scoringState = 102;
                    timeToggle3 = true;
                }
                break;
            case 102:
                pitchSlidesState = 100;
                pitch.setState(Pitch.PitchState.INITIALIZE);
                slides.setState(DepoSlides.DepositState.DOWN);
                if(timeToggle3){
                    TimeStamp3 = timer3.milliseconds();
                    timeToggle3 = false;
                }
                if(timer.milliseconds() > TimeStamp3 + 500){
                    scoringState = -1;
                    timeToggle3 = true;
                }
                thisToggle = true;
                break;
            default:
                scoringState = -1;
                break;
        }
    }
    public void updateGamepadTwo(){
        //gamepad 2
        if(gamepadTwo.left_bumper) autoIntake = !autoIntake;

        if(gamepadTwo.right_bumper){
            climbSafe++;
        }
        if(gamepadTwo.b){
            ledState++;
        }
        if(gamepadTwo.dpad_up) level++;
        if(gamepadTwo.dpad_down) level--;

        if(level > 5) level = 5;
        if(level < 0) level = 0;

        if(gamepadTwo.dpad_right) extension++;
        if(gamepadTwo.dpad_left) extension--;
        if(extension > 5) extension = 5;
        if(extension < 0) extension = 0;

        if(gamepadTwo.x){
            level = 0;
            extension = 0;
        }
        if(gamepadTwo.dpad_left){
            intake.setState(Intake.IntakeState.INCRIMENT_UP);
        }else if(gamepadTwo.dpad_right) {
            intake.setState(Intake.IntakeState.INCRIMENT_DOWN);
        }
    }
    public void updateVariables(){
        if(autoIntake)intake.setPower(-0.8);
        else intake.setPower((gamepad2.left_trigger - gamepad2.right_trigger)*1);

        if(climbSafe==3){
            pitch.setState(Pitch.PitchState.CLIMB);
            arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);
        }else if (climbSafe == 4){
            arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);
            pitch.manualMode = true;
            pitch.setPowerManual(-gamepad2.left_stick_y);
        }else if(climbSafe > 4){
            climbSafe = 0;
            pitch.manualMode = false;
        }
        if(ledState == 0){
            led.changePattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }else if(ledState == 1){
            led.changePattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        }else if(ledState == 2) {
            led.changePattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }else if(ledState == 3) {
            led.changePattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        }else if(ledState >3){
            ledState = 0;
        }
//        if(autoLockMode){ //only update this if in auto lock mode, otherwise its not needed THANKS DAD
//            adjustedAngle = drive.getPoseEstimate().getHeading() - boardAngle;
//            if(adjustedAngle > M.PI){
//                adjustedAngle -= 2*M.PI;
//            }else if(adjustedAngle < -M.PI){
//                adjustedAngle += 2* M.PI;
//            }
//            turret.robotAngle = adjustedAngle;
//        }
        pitch.level = this.level;
        arm.level = this.level;
        slides.level = this.level;
        slides.extension = this.extension;
    }
}
