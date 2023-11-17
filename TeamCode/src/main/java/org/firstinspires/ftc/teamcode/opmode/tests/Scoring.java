package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Pitch;
import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

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

    double TimeStamp = 0;

    boolean timeToggle = true;
    double adjustedAngle = 0;
    double boardAngle = 0;

    int height = 1;
    int scoringState = 0;


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

            if(autoIntake)intake.setPower(-0.8);
            else intake.setPower((gamepad1.left_trigger - gamepad1.right_trigger)*1);

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

            telemetryData();
            telemetry.update();
        }
    }

    public void telemetryData(){
        telemetry.addData("turret", turret.getState());
        telemetry.addData("arm", arm.getState());
        telemetry.addData("robot angle", drive.getPoseEstimate().getHeading());
        telemetry.addData("height", height); //2 is high, 1 is norm
        telemetry.addData("scoring State", scoringState);
        telemetry.addData("level", level);
        telemetry.addData("extension", extension);
        telemetry.addData("pitch", pitch.getState());
        telemetry.addData("slides", slides.getState());
        telemetry.addData("slidesTarget", slides.getCurrentPosition());
        telemetry.addData("adjusted angle", adjustedAngle);
        telemetry.addData("auto lock Mode", autoLockMode);
        telemetry.addData("AMPS(not including drive)", slides.leftMotor.getCurrentAMPS() + slides.rightMotor.getCurrentAMPS() + intake.intakeMotor.getCurrent(CurrentUnit.AMPS) + pitch.pitchMotor.getCurrentAMPS());
    }

    public void updateGamepadOne(){
        if(gamepadOne.y) autoLockMode = !autoLockMode;
        if(gamepadOne.right_stick_button) boardAngle = drive.getPoseEstimate().getHeading();

        if(gamepadOne.dpad_up) level++;
        if(gamepadOne.dpad_down) level--;

        if(level > 5) level = 5;
        if(level < 0) level = 0;

        if(gamepadOne.dpad_right) extension++;
        if(gamepadOne.dpad_left) extension--;
        if(extension > 5) extension = 5;
        if(extension < 0) extension = 0;

        if(gamepadOne.right_bumper) scoringState++;

        if(gamepadOne.left_bumper) autoIntake = !autoIntake;
    }
    public void scoringStateMachine(){
        switch (scoringState){
            case 0: //Init
                slides.setState(DepoSlides.DepositState.DOWN);
                pitch.setState(Pitch.PitchState.INITIALIZE);
                arm.setState(DepoArm.DepoArmState.TRANSFER);
                break;
            case 1: //arm / pitch / slides up
                pitch.setState(Pitch.PitchState.SCOREATLEVEL);
                arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);

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
                slides.setState(DepoSlides.DepositState.CALCULATED_UP);
                if(autoLockMode) {
                    //turret.robotAngle =
                    turret.setState(LM1Turret.TurretState.AUTOLOCK);
                }
                else turret.setState(LM1Turret.TurretState.SCORE);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds()> TimeStamp + 300){
                    arm.setState(DepoArm.DepoArmState.INTERMEDIATE);
                    timeToggle = true;
                }

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
                slides.setState(DepoSlides.DepositState.DOWN);
                if(slides.getCurrentPosition() > 0.2){
                    break;
                }
                pitch.setState(Pitch.PitchState.INITIALIZE);
                if(timeToggle){
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds()>TimeStamp + 500){
                    scoringState=0;
                    timeToggle=true;
                }

                break;
            default:
                scoringState = 0;
                break;
        }
    }
    public void updateGamepadTwo(){
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
    }
    public void updateVariables(){
        if(autoLockMode){ //only update this if in auto lock mode, otherwise its not needed THANKS DAD
            adjustedAngle = drive.getPoseEstimate().getHeading() - boardAngle;
            if(adjustedAngle > M.PI){
                adjustedAngle -= 2*M.PI;
            }else if(adjustedAngle < -M.PI){
                adjustedAngle += 2* M.PI;
            }
            turret.robotAngle = adjustedAngle;
        }
        pitch.level = this.level;
        arm.level = this.level;
        slides.level = this.level;
        slides.extension = this.extension;
    }
}
